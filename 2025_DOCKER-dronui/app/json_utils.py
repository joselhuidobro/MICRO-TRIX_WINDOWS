import os
import struct
import re
import time
import threading
import json as pyjson
import socket
import docker
from collections import deque
from queue import Empty
from flask import Flask, Response, render_template, jsonify, request
# --------- JSON ultrarrápido si existe ---------
try:
    import orjson
    def fast_dumps(obj):
        # orjson devuelve bytes; SSE requiere str
        return orjson.dumps(obj, option=orjson.OPT_APPEND_NEWLINE).decode()[:-1]
except Exception:
    def fast_dumps(obj):
        return pyjson.dumps(obj, separators=(",", ":"))
import rclpy
from rclpy.node import Node
from std_msgs.msg import String as RosString
# ---------- Flask ----------
app = Flask(__name__, static_folder="static", template_folder="templates")
# ---------- Config ----------
ROS_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "10"))
UDP_PORT = int(os.environ.get("UDP_PORT", "5002"))
# ---------- Estado compartido ----------
latest = {
    "pitch": 0.0, "roll": 0.0, "yaw": 0.0,
    "m1": 0, "m2": 0, "m3": 0, "m4": 0,
    "t_us": None,
    "lat_ms": None
}
# ---------- Señalización ultra-low-latency para SSE ----------
_updates_q = deque(maxlen=256) # mantenemos un búfer pequeño
_updates_evt = threading.Event() # despierta al generador SSE al instante
_lock_latest = threading.Lock() # protege 'latest' en lecturas/escrituras
def _publish_update(payload: dict):
    with _lock_latest:
        latest.update(payload)
    _updates_q.append(payload)
    _updates_evt.set()
# ---------- Parser del String publicado por el ESP32 (ROS) ----------
ANG_RE = re.compile(
    r"Ang:\s*(-?\d+(?:\.\d+)?)\/(-?\d+(?:\.\d+)?)\/(-?\d+(?:\.\d+)?)\s*\|\s*M:\s*(\d+)\s+(\d+)\s+(\d+)\s+(\d+)"
)
def parse_telem_string(s: str):
    m = ANG_RE.search(s)
    if not m:
        return None
    pitch, roll, yaw, m1, m2, m3, m4 = m.groups()
    return {
        "pitch": float(pitch),
        "roll": float(roll),
        "yaw": float(yaw),
        "m1": int(m1),
        "m2": int(m2),
        "m3": int(m3),
        "m4": int(m4),
        "t_us": None,
        "lat_ms": None,
    }
# ---------- Telemetría UDP (binaria) ----------
# Precompilamos Structs para minimizar trabajo en hot-path
_PACK_NO_TS = struct.Struct("<3f4h") # 3 floats + 4 int16
_PACK_TS = struct.Struct("<3f4hI") # + uint32 (t_us)
LEN_NO_TS = _PACK_NO_TS.size
LEN_TS = _PACK_TS.size
MAX_LEN = max(LEN_TS, LEN_NO_TS)
def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # Reutilización y buffer grande para evitar drops
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    except OSError:
        pass
    # Tiempo de espera muy pequeño para poder salir limpiamente si se desea
    # (pero seguimos en modo "bloqueante" por latencia mínima)
    try:
        sock.bind(("", UDP_PORT))
        print(f"[UDP] Listener en 0.0.0.0:{UDP_PORT}")
    except Exception as e:
        print(f"[UDP] Error bind: {e}")
        return
    buf = bytearray(MAX_LEN)
    view = memoryview(buf)
    while True:
        try:
            n, addr = sock.recvfrom_into(view, MAX_LEN)
            # Detecta si viene con timestamp o no
            if n >= LEN_TS:
                pitch, roll, yaw, m1, m2, m3, m4, t_us = _PACK_TS.unpack_from(view)
            elif n >= LEN_NO_TS:
                pitch, roll, yaw, m1, m2, m3, m4 = _PACK_NO_TS.unpack_from(view)
                t_us = None
            else:
                continue
            # Timestamp actual en µs (monotónico ≈ mejor para latencias)
            now_us = time.time_ns() // 1_000
            lat_ms = None
            if t_us is not None:
                # t_us es uint32 del ESP (wrap cada ~71 min). Usamos diferencia simple.
                dt_us = (now_us - int(t_us)) & 0xFFFFFFFFFFFFFFFF # seguro
                lat_ms = round(max(0, dt_us) / 1000.0, 2)
            payload = {
                "pitch": float(pitch),
                "roll": float(roll),
                "yaw": float(yaw),
                "m1": int(m1), "m2": int(m2), "m3": int(m3), "m4": int(m4),
                "t_us": int(t_us) if t_us is not None else None,
                "lat_ms": lat_ms
            }
            _publish_update(payload) # despierta SSE inmediatamente
        except struct.error:
            # Silenciamos errores de desempaquetado en paquetes corruptos
            continue
        except Exception as e:
            # Log sin spamear
            print(f"[UDP] Error: {e}")
            time.sleep(0.005)
# ---------- ROS 2 subscriber ----------
class TelemetrySubscriber(Node):
    def __init__(self):
        super().__init__('ui_telemetry_subscriber')
        self.create_subscription(RosString, 'drone_telemetry', self.cb, 10)
    def cb(self, msg: RosString):
        print(f"[ROS] Received raw message: {msg.data}")
        data = parse_telem_string(msg.data)
        if not data:
            print("[ROS] Parse failed for the received message.")
            return
        print(f"[ROS] Parsed data: {data}")
        # No pisamos si UDP ya alimenta; pero sí publicamos a UI para redundancia
        _publish_update(data)
def ros_thread():
    os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
    try:
        rclpy.init()
    except Exception as e:
        print(f"[ROS] rclpy.init() falló: {e}")
        return
    node = TelemetrySubscriber()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02) # más reactivo
    except Exception as e:
        print(f"[ROS] Loop error: {e}")
    finally:
        try: node.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass
# ---------- Rutas Flask ----------
@app.route("/")
def index():
    with _lock_latest:
        snap = dict(latest)
    return render_template("index.html", telemetry=snap)
@app.route("/orientation")
def orientation_stream():
    def sse_bytes(payload: dict) -> bytes:
        # Arma un evento SSE y lo codifica a bytes
        return ("data:" + fast_dumps(payload) + "\n\n").encode("utf-8")
    def gen():
        heartbeat_every = 2.0
        last_sent = time.time()
        # Muestra inicial
        with _lock_latest:
            snap = dict(latest)
        yield sse_bytes(snap)
        while True:
            fired = _updates_evt.wait(timeout=heartbeat_every)
            if fired:
                _updates_evt.clear()
                last = None
                while _updates_q:
                    last = _updates_q.pop()
                    _updates_q.clear()
                if last is not None:
                    yield sse_bytes(last)
                    last_sent = time.time()
            else:
                # Heartbeat como bytes
                yield b":\n\n"
                last_sent = time.time()
    return Response(
        gen(),
        # mejor ser explícitos con el charset
        content_type="text/event-stream; charset=utf-8",
        headers={
            "Cache-Control": "no-cache",
            "X-Accel-Buffering": "no",
            "Connection": "keep-alive",
        },
        # puedes quitarlo si quieres; bytes ya evita el problema
        direct_passthrough=True
    )
# ----- Acciones al contenedor 'robot' -----
@app.route('/build_idf', methods=['POST'])
def build_idf():
    try:
        client = docker.from_env()
        container = client.containers.get('robot')
        cmd = 'bash -c "source /opt/esp-idf/export.sh >/dev/null && cd /esp2024 && idf.py build"'
        exec_result = container.exec_run(cmd=cmd, workdir='/esp2024')
        output = (exec_result.output or b"").decode(errors="ignore")
        if exec_result.exit_code == 0:
            return jsonify(status="success", message="Build completado.", output=output)
        return jsonify(status="error", message="Build falló.", output=output), 400
    except docker.errors.APIError as e:
        return jsonify(status="error", message=f"Docker API error: {str(e)}"), 500
    except Exception as e:
        return jsonify(status="error", message=f"Error ejecutando build: {str(e)}"), 500
@app.route('/flash_esp32', methods=['POST'])
def flash_esp32():
    port = request.json.get('port', '/dev/ttyUSB0') if request.is_json else '/dev/ttyUSB0'
    try:
        client = docker.from_env()
        container = client.containers.get('robot')
        cmd = f'bash -c "source /opt/esp-idf/export.sh >/dev/null && cd /esp2024 && idf.py -p {port} flash"'
        exec_result = container.exec_run(cmd=cmd, workdir='/esp2024')
        output = (exec_result.output or b"").decode(errors="ignore")
        if exec_result.exit_code == 0:
            return jsonify(status="success", message=f"Flash OK en {port}.", output=output)
        return jsonify(status="error", message="Flash falló.", output=output), 400
    except docker.errors.APIError as e:
        return jsonify(status="error", message=f"Docker API error: {str(e)}"), 500
    except Exception as e:
        return jsonify(status="error", message=f"Error ejecutando flash: {str(e)}"), 500


@app.route('/idf_monitor', methods=['POST'])
def idf_monitor():
    port = request.json.get('port', '/dev/ttyUSB0') if request.is_json else '/dev/ttyUSB0'
    try:
        client = docker.from_env()
        container = client.containers.get('robot')
        cmd = f'bash -c "source /opt/esp-idf/export.sh >/dev/null && cd /esp2024 && idf.py -p {port} monitor"'
        exec_result = container.exec_run(cmd=cmd, workdir='/esp2024')
        output = (exec_result.output or b"").decode(errors="ignore")
        return jsonify(status="success", message=f"Monitor lanzado en {port}.", output=output)
    except docker.errors.APIError as e:
        return jsonify(status="error", message=f"Docker API error: {str(e)}"), 500
    except Exception as e:
        return jsonify(status="error", message=f"Error iniciando monitor: {str(e)}"), 500


@app.route('/get_code', methods=['POST'])
def get_code():
    return jsonify(status="success", message="Codigo button acknowledged.")
# ---------- Main ----------
if __name__ == "__main__":
    # Hilos: UDP + ROS
    threading.Thread(target=udp_listener, daemon=True).start()
    threading.Thread(target=ros_thread, daemon=True).start()
    # Para DEV: el server de Flask no es ideal para SSE; usa gunicorn+gevent en prod.
    app.run(host="0.0.0.0", port=5500, threaded=True)
