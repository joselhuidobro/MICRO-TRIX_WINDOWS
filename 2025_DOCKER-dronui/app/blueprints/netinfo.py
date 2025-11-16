# app/blueprints/netinfo.py
import time, subprocess
from flask import Blueprint, jsonify, Response

bp = Blueprint("netinfo", __name__)

def _safe(cmd):
    try:
        return subprocess.check_output(cmd, shell=True, timeout=1.5).decode().strip()
    except Exception:
        return ""

def build_net_info():
    def _safe(cmd, tout=1.5):
        try:
            import subprocess
            return subprocess.check_output(cmd, shell=True, timeout=tout).decode().strip()
        except Exception:
            return ""

    # IP pública (IPv4 / IPv6)
    public4 = _safe('curl -s https://api.ipify.org')
    public6_raw = _safe('curl -s https://api64.ipify.org')
    public6 = public6_raw if ":" in public6_raw else None

    # LAN IPv4 (del contenedor, IP usada para salir a Internet por default route)
    # "ip route get 1" → ... src <IP> ...
    lan_ip = _safe("ip route get 1 | awk '{for(i=1;i<=NF;i++) if($i==\"src\") print $(i+1)}'")

    # Bridge Docker (IP del contenedor en eth0, típicamente 172.18.x.x)
    docker_eth0 = _safe("ip -4 addr show eth0 | awk '/inet /{print $2}' | cut -d/ -f1")

    # WireGuard (solo si existe wg0 dentro de ESTE contenedor)
    wg_ip   = _safe("ip -4 addr show wg0 | awk '/inet /{print $2}' | cut -d/ -f1")
    wg_ep   = None  # si quieres, llénalo leyendo de Redis/Prometheus o via Docker SDK -> wireguard wg show

    lan_list = [ip for ip in [lan_ip] if ip]

    return {
        "public_ipv4": public4 or None,
        "public_ipv6": public6,
        "wg": {"server_ip": wg_ip or None, "endpoint": wg_ep},
        "lan": {"ipv4": lan_list},                 # ahora no vacío
        "docker": {"bridge_ipv4": docker_eth0 or None},
        "ts": __import__("time").time().__int__(),
        "admin": False,
    }


@bp.get("/net/info")
def net_info():
    try:
        return jsonify(build_net_info()), 200, {"Cache-Control":"no-store"}
    except Exception as e:
        return jsonify({"status":"error","message":str(e)}), 503, {"Cache-Control":"no-store"}

# opcional: /ui/net/public-ip (si lo quieres)
@bp.get("/net/public-ip")
def public_ip():
    return jsonify({
        "ipv4": _safe('curl -s https://api.ipify.org') or None,
        "ipv6": _safe('curl -s https://api64.ipify.org') or None,
        "ts": int(time.time())
    }), 200, {"Cache-Control":"no-store"}

