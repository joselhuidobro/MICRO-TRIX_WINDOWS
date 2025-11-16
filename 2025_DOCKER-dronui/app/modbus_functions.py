# app/modbus_functions.py
import socket
import concurrent.futures
from pymodbus.client import ModbusTcpClient
from pymodbus.exceptions import ModbusException
import struct
import subprocess
import ipaddress
import sys
import re
import os
from typing import Optional

from config import MODBUS_HOST, MODBUS_PORT, MODBUS_UNIT, MODBUS_ENABLED

if sys.platform != 'win32':
    import fcntl

# ====== Config de robustez / compat ======
MODBUS_TIMEOUT = float(os.getenv("MODBUS_TIMEOUT", "0.6"))      # tiempo de espera corto
SCAN_TIMEOUT   = float(os.getenv("MODBUS_SCAN_TIMEOUT", "0.4")) # timeout por host en escaneo

# Escritura tolerante: holding register / coil / auto
LW_WRITE_MODE  = os.getenv("LW_WRITE_MODE", "register").lower()  # register | coil | auto
COIL_ADDR_BASE = int(os.getenv("COIL_ADDR_BASE", "0"))

# Caché local de estados de relés (para publicar cambios y modo offline)
_last_lw = {"lw0": None, "lw1": None, "lw2": None}


# ====== Helpers ROS 2 ======
def _relay_name_from_bit(bit: int) -> str:
    return {0: "relay1", 1: "relay2", 2: "relay3"}.get(bit, f"relay{bit+1}")

def _ros2_publish_string(message: str, topic: str = "/relevadores"):
    """Publica una vez a ROS2 usando el CLI. Devuelve (ok, salida_ó_error). Nunca lanza excepción afuera."""
    try:
        if sys.platform == "win32":
            cmd = ["ros2", "topic", "pub", "--once", topic, "std_msgs/msg/String", f"data: {message}"]
            cp = subprocess.run(cmd, check=True, capture_output=True, text=True)
            return True, cp.stdout.strip()
        else:
            cmd = (
                'source /opt/ros/${ROS_DISTRO:-humble}/setup.bash; '
                f'ros2 topic pub --once {topic} std_msgs/msg/String "data: {message}"'
            )
            cp = subprocess.run(["bash", "-lc", cmd], check=True, capture_output=True, text=True)
            return True, cp.stdout.strip()
    except Exception as e:
        return False, str(e)


# ====== Compat helpers para distintas versiones de pymodbus ======
def _mb_call(func, **kwargs):
    """Intenta llamar a `func` con unit/slave/device_id según la versión instalada."""
    for key in ("unit", "slave", "device_id"):
        try:
            return func(**{**kwargs, key: MODBUS_UNIT})
        except TypeError:
            continue
    # Último intento sin unidad (firmas muy viejas)
    return func(**kwargs)

def _new_client() -> ModbusTcpClient:
    """Crea cliente con timeouts cortos; evita kwargs incompatibles en versiones antiguas."""
    try:
        return ModbusTcpClient(host=MODBUS_HOST, port=MODBUS_PORT, timeout=MODBUS_TIMEOUT)
    except TypeError:
        # Versiones antiguas con args posicionales
        return ModbusTcpClient(MODBUS_HOST, MODBUS_PORT)


# ====== Red / Escaneo ======
def get_network_info():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(0.5)
        s.connect(("8.8.8.8", 80))
        local_ip = s.getsockname()[0]
        s.close()

        netmask = None
        network_str = None
        hosts = None

        if sys.platform == 'win32':
            try:
                output = subprocess.check_output('ipconfig /all', timeout=1.5).decode(errors='replace')
                adapters = re.split(r'\n\s*\n', output)
                ipv4_pattern = re.compile(r'IPv4 Address.*:\s*([\d.]+)')
                subnet_pattern = re.compile(r'Subnet Mask.*:\s*([\d.]+)')
                for adapter in adapters:
                    if local_ip in adapter:
                        ipv4_match = ipv4_pattern.search(adapter)
                        subnet_match = subnet_pattern.search(adapter)
                        if ipv4_match and subnet_match and ipv4_match.group(1) == local_ip:
                            netmask = subnet_match.group(1)
                            break
            except Exception:
                pass
        else:
            ifname: Optional[str] = None
            try:
                output = subprocess.check_output(["ip", "route", "show", "default"], timeout=1.0).decode()
                for line in output.split('\n'):
                    if line.startswith('default'):
                        parts = line.split()
                        dev_index = parts.index('dev') if 'dev' in parts else -1
                        if dev_index != -1:
                            ifname = parts[dev_index + 1]
                            break
            except Exception:
                pass

            if ifname:
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                    ifreq = struct.pack('256s', ifname[:15].encode('utf-8'))
                    res = fcntl.ioctl(sock.fileno(), 0x891b, ifreq)
                    netmask = socket.inet_ntoa(res[20:24])
                except Exception:
                    pass

        if netmask:
            try:
                iface = ipaddress.IPv4Interface(f"{local_ip}/{netmask}")
                network = iface.network
                network_str = str(network)
                hosts = [str(ip) for ip in network.hosts()]
            except Exception:
                pass

        if not hosts:
            parts = local_ip.split('.')
            network_str = '.'.join(parts[:3]) + '.0/24'
            netmask = '255.255.255.0'
            hosts = ['.'.join(parts[:3]) + '.' + str(i) for i in range(1, 255)]

        return {
            'local_ip': local_ip,
            'network': network_str,
            'netmask': netmask,
            'hosts_to_scan': hosts
        }
    except Exception:
        return {
            'local_ip': 'unknown',
            'network': '192.168.1.0/24',
            'netmask': '255.255.255.0',
            'hosts_to_scan': ['192.168.1.' + str(i) for i in range(1, 255)]
        }

def scan_modbus(ip):
    try:
        client = ModbusTcpClient(host=ip, port=MODBUS_PORT, timeout=SCAN_TIMEOUT)
        try:
            if client.connect():
                try:
                    r = _mb_call(client.read_coils, address=0, count=1)
                    # Si hay respuesta (error o no), hay servidor Modbus vivo
                    return ip if r is not None else None
                except ModbusException:
                    return ip
            return None
        finally:
            try: client.close()
            except Exception: pass
    except Exception:
        return None


# ====== Helpers internos ======
def _def_resp(last_error=None):
    """Respuesta estable cuando no hay HMI; mezcla el caché para ver toggles en modo offline."""
    base = {
        'lb100': False,
        'lb103': False,
        'lb104': False,
        'lb105': False,
        'rb1_0': False,
        'lw0': 0,
        'lw1': 0,
        'lw2': 0,
        'hmi_online': False,
    }
    # Mezcla caché local para que la UI refleje toggles aún sin HMI
    for idx in (0, 1, 2):
        v = _last_lw.get(f"lw{idx}")
        if isinstance(v, (int, bool)):
            base[f"lw{idx}"] = int(v)
    if last_error:
        base['last_error'] = str(last_error)
    return base

def _try_write_register(client, address, value):
    try:
        w = _mb_call(client.write_register, address=address, value=value)
        return bool(w) and not w.isError()
    except Exception:
        return False

def _try_write_coil(client, coil_addr, value_bool):
    try:
        w = _mb_call(client.write_coil, address=coil_addr, value=bool(value_bool))
        return bool(w) and not w.isError()
    except Exception:
        return False


# ====== Lectura/Escritura Modbus ======
def get_modbus_data():
    """
    Lee estados del HMI. Si detecta cambios en lw0/lw1/lw2 desde la última lectura,
    publica a ROS2 'relayN_on|off'. Nunca lanza excepción: siempre retorna un dict estable.
    """
    client = _new_client()
    try:
        if not client.connect():
            return _def_resp("No se pudo conectar al HMI")

        lb100 = lb103 = lb104 = lb105 = False
        rb1_0 = False
        lw0 = lw1 = lw2 = 0

        # ---- Lectura de coils ----
        try:
            r = _mb_call(client.read_coils, address=100, count=4)  # 100..103
            if r and not r.isError():
                bits = r.bits or []
                lb100 = bool(bits[0]) if len(bits) > 0 else False
                lb103 = bool(bits[3]) if len(bits) > 3 else False

            r2 = _mb_call(client.read_coils, address=104, count=2)  # 104..105
            if r2 and not r2.isError():
                bits2 = r2.bits or []
                lb104 = bool(bits2[0]) if len(bits2) > 0 else False
                lb105 = bool(bits2[1]) if len(bits2) > 1 else False
        except Exception:
            pass

        # ---- Lectura de registros ----
        try:
            r_rw1 = _mb_call(client.read_holding_registers, address=1, count=1)
            if r_rw1 and not r_rw1.isError():
                rw1 = r_rw1.registers[0]
                rb1_0 = (rw1 & 0x0001) != 0
        except Exception:
            pass

        try:
            r_lw = _mb_call(client.read_holding_registers, address=0, count=3)  # lw0..lw2
            if r_lw and not r_lw.isError():
                regs = r_lw.registers or [0, 0, 0]
                lw0 = int(regs[0]) if len(regs) > 0 else 0
                lw1 = int(regs[1]) if len(regs) > 1 else 0
                lw2 = int(regs[2]) if len(regs) > 2 else 0
        except Exception:
            pass

        # ==== Detección de cambios y publish ROS (solo si no es la primera lectura) ====
        try:
            for idx, val in enumerate([lw0, lw1, lw2]):
                key = f"lw{idx}"
                prev = _last_lw[key]
                if isinstance(val, (int, bool)):
                    if prev is None:
                        _last_lw[key] = int(val)  # inicializa sin publicar
                    else:
                        val_int = int(val)
                        if val_int != prev:
                            relay_name = _relay_name_from_bit(idx)
                            state = "on" if val_int != 0 else "off"
                            _ros2_publish_string(f"{relay_name}_{state}", topic="/relevadores")
                            _last_lw[key] = val_int
        except Exception as e:
            print(f"[WARN] Error al publicar cambio ROS: {e}")

        return {
            'lb100': lb100,
            'lb103': lb103,
            'lb104': lb104,
            'lb105': lb105,
            'rb1_0': rb1_0,
            'lw0': lw0,
            'lw1': lw1,
            'lw2': lw2,
            'hmi_online': True
        }

    except ModbusException as e:
        return _def_resp(e)
    except Exception as e:
        return _def_resp(e)
    finally:
        try: client.close()
        except Exception: pass


def toggle_lw(address: int):
    """
    Toggle de LW{address}. Si el HMI no aplica, se publica a ROS igual (modo degradado)
    y se actualiza el caché local, para que el resto del sistema siga operando.
    Nunca lanza excepción: regresa dict con status.
    """
    client = _new_client()
    try:
        if client.connect():
            # 1) Leer estado actual (según modo)
            current = 0
            read_ok = False

            if LW_WRITE_MODE in ("register", "auto"):
                r = _mb_call(client.read_holding_registers, address=address, count=1)
                if r and not r.isError():
                    current = int(r.registers[0])
                    read_ok = True

            if not read_ok and LW_WRITE_MODE in ("coil", "auto"):
                rc = _mb_call(client.read_coils, address=COIL_ADDR_BASE + address, count=1)
                if rc and not rc.isError() and rc.bits:
                    current = 1 if rc.bits[0] else 0
                    read_ok = True

            new_value = 1 if int(current) == 0 else 0

            # 2) Intentar escribir (según modo)
            applied = False
            path = None

            if LW_WRITE_MODE in ("register", "auto"):
                if _try_write_register(client, address, new_value):
                    applied = True
                    path = "register"

            if not applied and LW_WRITE_MODE in ("coil", "auto"):
                coil_addr = COIL_ADDR_BASE + address
                if _try_write_coil(client, coil_addr, new_value):
                    applied = True
                    path = "coil"

            # 3) Publicar a ROS SIEMPRE (aplicado o degradado)
            relay_name = _relay_name_from_bit(address)
            state = "on" if new_value == 1 else "off"
            ros_message = f"{relay_name}_{state}"
            ok, out = _ros2_publish_string(ros_message, topic="/relevadores")

            # Actualiza caché para evitar doble publish en la siguiente lectura
            try:
                _last_lw[f"lw{address}"] = int(new_value)
            except Exception:
                pass

            if applied:
                return {
                    'status': 'success',
                    'message': f'Lw{address} toggled (HMI aplicado por {path})',
                    'new_value': new_value,
                    'hmi_applied': True,
                    'hmi_path': path,
                    'ros_published': ok,
                    'ros_message': ros_message,
                    'ros_output': out,
                }
            else:
                # MODO DEGRADADO: no se pudo escribir, pero mantenemos operación por ROS
                return {
                    'status': 'degraded',
                    'message': f'Lw{address} toggled (HMI no aplicó; operación por ROS)',
                    'new_value': new_value,
                    'hmi_applied': False,
                    'ros_published': ok,
                    'ros_message': ros_message,
                    'ros_output': out,
                }

        # Sin conexión: degradado directo
        relay_name = _relay_name_from_bit(address)
        new_value = 1 if int(_last_lw.get(f"lw{address}", 0) or 0) == 0 else 0
        state = "on" if new_value == 1 else "off"
        ros_message = f"{relay_name}_{state}"
        ok, out = _ros2_publish_string(ros_message, topic="/relevadores")
        try:
            _last_lw[f"lw{address}"] = int(new_value)
        except Exception:
            pass
        return {
            'status': 'degraded',
            'message': 'HMI no disponible (conexión fallida); operación por ROS',
            'new_value': new_value,
            'hmi_applied': False,
            'ros_published': ok,
            'ros_message': ros_message,
            'ros_output': out,
        }

    except Exception as e:
        # Cualquier error inesperado -> también degradado (no rompemos la app)
        relay_name = _relay_name_from_bit(address)
        new_value = 1 if int(_last_lw.get(f"lw{address}", 0) or 0) == 0 else 0
        state = "on" if new_value == 1 else "off"
        ros_message = f"{relay_name}_{state}"
        ok, out = _ros2_publish_string(ros_message, topic="/relevadores")
        try:
            _last_lw[f"lw{address}"] = int(new_value)
        except Exception:
            pass
        return {
            'status': 'degraded',
            'message': f'Error HMI: {str(e)}; operación por ROS',
            'new_value': new_value,
            'hmi_applied': False,
            'ros_published': ok,
            'ros_message': ros_message,
            'ros_output': out,
        }
    finally:
        try: client.close()
        except Exception: pass


# ====== Escaneo ======
def perform_modbus_scan():
    info = get_network_info()
    ips = info['hosts_to_scan']
    found = []
    with concurrent.futures.ThreadPoolExecutor(max_workers=64) as executor:
        future_to_ip = {executor.submit(scan_modbus, ip): ip for ip in ips}
        for future in concurrent.futures.as_completed(future_to_ip):
            result = future.result()
            if result:
                found.append(result)
    message = (
        f"Local IPv4: {info['local_ip']}\n"
        f"Segmento de red: {info['network']}\n"
        f"Máscara de subred: {info['netmask']}\n"
    )
    if found:
        message += 'Dispositivos Modbus encontrados en: ' + ', '.join(found)
        status = 'success'
    else:
        message += 'No se encontraron dispositivos Modbus'
        status = 'error'
    return {'status': status, 'message': message}

