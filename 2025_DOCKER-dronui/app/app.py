import os, sys, logging, uuid, time, threading, docker, json, traceback, socket, random
from queue import Queue, Full, Empty
from typing import List, Dict

from flask import Flask, Response, render_template, jsonify, request, g
from flask_sock import Sock

from prometheus_client import Counter, Histogram, generate_latest, CONTENT_TYPE_LATEST

import config  # ← ahora importamos el módulo completo


from blueprints import netinfo_bp

from modbus_functions import get_modbus_data, toggle_lw, perform_modbus_scan
from ros_utils import ros_thread
from rt_bus import publish_update, snapshot, sse_stream
from udp_service import start_udp_listener
from modbus_service import start_modbus_poller

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger

# Logging JSON unificado
from logging_setup import setup_json_logging
setup_json_logging(os.environ.get("LOG_LEVEL","INFO"))

# ===== Radar UDP =====
from udp_discovery import UDPRadar
from ros_monitor import init_ros_monitor   # ← NUEVO

# ===== Redis =====
try:
    import redis  # pip install redis
except Exception:
    redis = None

HTTP_REQ_COUNTER = Counter("ui_http_requests_total", "Total de requests", ["method", "path", "code"])
HTTP_LATENCY = Histogram("ui_http_request_duration_seconds", "Latencia HTTP", ["method", "path"])

REDIS_HOST = os.environ.get("REDIS_HOST", "redis")
REDIS_PORT = int(os.environ.get("REDIS_PORT", "6379"))
REDIS_PASSWORD = os.environ.get("REDIS_PASSWORD", "")
REDIS_DB = int(os.environ.get("REDIS_DB", "0"))

REDIS_KEYS = {
    "modbus_last": "trix:modbus:last",
    "udp_devices_last": "trix:udp:devices:last",   # ← escribe el net-sensor (LAN)
    "snapshot_last": "trix:snapshot:last",
    "updates_stream": "trix:updates",
    "updates_chan": "trix:updates:chan",
}

# ===== Flask app =====
app = Flask(__name__, static_folder="static", template_folder="templates")

app.register_blueprint(netinfo_bp, url_prefix="/api")


sock = Sock(app)
ros_publish_q = Queue()
init_ros_monitor(app)                  # ← NUEVO: agrega /ros/graph y /ros/graph/stream

# ===== Redis helpers =====
_rds = None
def _redis_connect():
    global _rds
    if redis is None:
        return None
    if _rds:
        return _rds
    try:
        _rds = redis.Redis(
            host=REDIS_HOST,
            port=REDIS_PORT,
            password=(REDIS_PASSWORD or None),
            db=REDIS_DB,
            socket_timeout=2.0,
            socket_connect_timeout=2.0,
            health_check_interval=30,
            retry_on_timeout=True,
            decode_responses=True,
        )
        _rds.ping()
        app.logger.info("[REDIS] conectado", extra={"subsystem": "REDIS", "host": REDIS_HOST, "port": REDIS_PORT})
        return _rds
    except Exception as e:
        try:
            app.logger.warning("[REDIS] no disponible: %s", str(e), extra={"subsystem":"REDIS"})
        except Exception:
            pass
        return None

def rset_json(key: str, obj, ex: int | None = None):
    r = _redis_connect()
    if not r:
        return False
    try:
        payload = json.dumps(obj, ensure_ascii=False)
        return bool(r.set(key, payload, ex=ex))
    except Exception:
        app.logger.exception("[REDIS] set_json(%s) falló", key, extra={"subsystem":"REDIS"})
        return False

def rget_json(key: str, default=None):
    r = _redis_connect()
    if not r:
        return default
    try:
        raw = r.get(key)
        if raw is None:
            return default
        obj = json.loads(raw)
        # Si todavía es str, intenta una segunda carga (doble JSON)
        if isinstance(obj, str):
            try:
                obj2 = json.loads(obj)
                return obj2
            except Exception:
                return obj
        return obj
    except Exception:
        app.logger.exception("[REDIS] get_json(%s) falló", key, extra={"subsystem":"REDIS"})
        return default


def rpublish(channel: str, obj) -> None:
    r = _redis_connect()
    if not r:
        return
    try:
        r.publish(channel, json.dumps(obj, ensure_ascii=False))
    except Exception:
        app.logger.exception("[REDIS] publish(%s) falló", channel, extra={"subsystem":"REDIS"})

def rxadd(stream: str, obj, maxlen: int = 1000) -> None:
    r = _redis_connect()
    if not r:
        return
    try:
        r.xadd(stream, {"json": json.dumps(obj, ensure_ascii=False)}, maxlen=maxlen, approximate=True)
    except Exception:
        app.logger.exception("[REDIS] xadd(%s) falló", stream, extra={"subsystem":"REDIS"})

# ===== Logging helpers / sampling =====
LOG_LEVEL = os.environ.get("LOG_LEVEL", "INFO").upper()
level = getattr(logging, LOG_LEVEL, logging.INFO)

_last_log = {"GET /data": 0, "GET /telemetry": 0, "GET /udp/devices": 0}
MIN_GAP = 2.0  # segundos

def _should_sample(method, path):
    key = f"{method} {path.split('?')[0]}"
    now = time.time()
    gap = now - _last_log.get(key, 0)
    if key in _last_log and gap < MIN_GAP:
        return False
    _last_log[key] = now
    return True

def _client_ip():
    return request.headers.get("X-Forwarded-For", request.remote_addr or "-")

def _log_info(msg, **extra):
    rid = getattr(g, "req_id", None)
    if rid:
        extra.setdefault("req_id", rid)
    extra.setdefault("subsystem", "HTTP")
    app.logger.info(msg, extra=extra)

@app.before_request
def _start_timer():
    request._prom_start = HTTP_LATENCY.labels(request.method, request.path).time()

@app.after_request
def _after(response):
    try:
        request._prom_start()
    except Exception:
        pass
    HTTP_REQ_COUNTER.labels(request.method, request.path, str(response.status_code)).inc()
    return response

@app.route("/metrics")
def metrics():
    return Response(generate_latest(), mimetype=CONTENT_TYPE_LATEST)

# -------- Request logs --------
@app.before_request
def _log_req():
    g.req_id = request.headers.get("X-Request-ID") or uuid.uuid4().hex[:8]
    g.t0 = time.time()

    qs = request.query_string.decode() if request.query_string else ''
    path_qs = f"{request.path}?{qs}" if qs else request.path

    if request.method == "GET" and not _should_sample(request.method, request.path):
        pass
    else:
        _log_info(f">>> {request.method} {path_qs}",
                  method=request.method,
                  path=request.path,
                  query=qs,
                  ip=_client_ip(),
                  ct=request.headers.get("Content-Type",""))

    if request.method in ("POST","PUT","PATCH"):
        try:
            body = request.get_data(cache=True)
            if body:
                prev = body.decode("utf-8","ignore")
                if len(prev) > 600:
                    prev = prev[:600] + f"...(+{len(prev)-600} bytes)"
                _log_info(">>> body", body_preview=prev)
        except Exception as e:
            _log_info(">>> body-read-error", error=str(e))

@app.after_request
def _log_resp(resp):
    try:
        dt_ms = (time.time()-getattr(g,"t0",time.time()))*1000.0
    except Exception:
        dt_ms = 0.0
    try:
        if not (request.method == "GET" and not _should_sample(request.method, request.path)):
            _log_info(f"<<< {request.method} {request.path} -> {resp.status}",
                      status=resp.status_code,
                      bytes=resp.calculate_content_length(),
                      dt_ms=round(dt_ms,1))
    except Exception as e:
        _log_info("after_request log failed", error=str(e))
    return resp

@app.get("/health")
def health():
    r_ok = False
    try:
        r = _redis_connect()
        if r:
            r_ok = bool(r.ping())
    except Exception:
        r_ok = False
    return jsonify(status="ok", redis=r_ok), 200

# ======== MODBUS healthcheck ========
def _tcp_check(host: str, port: int, timeout=0.6) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except Exception:
        return False

@app.get("/modbus/health")
def modbus_health():
    ok = _tcp_check(config.MODBUS_HOST, config.MODBUS_PORT, timeout=0.6)
    return jsonify({
        "enabled": config.MODBUS_ENABLED,
        "host": config.MODBUS_HOST,
        "port": config.MODBUS_PORT,
        "unit": config.MODBUS_UNIT,
        "tcp_reachable": ok
    }), (200 if ok else 503)

# =========================
# ==== DEBUG / DIAGNÓSTICO
# =========================

def _safe_parse_json_maybe_str(obj):
    """Si obj es str con JSON dentro (doble-JSON), intenta cargarlo una vez."""
    if isinstance(obj, str):
        try:
            return json.loads(obj), True
        except Exception:
            return obj, False
    return obj, False

@app.get("/debug/redis-key")   # ← único endpoint; NO duplicarlo
def debug_redis_key():
    """
    Inspecciona la clave del sensor (LAN) en Redis:
      - tipo real (dict / str->json / str)
      - número de devices
    """
    raw = rget_json(REDIS_KEYS["udp_devices_last"])
    typ = type(raw).__name__
    parsed = raw
    double = False
    if isinstance(raw, str):
        parsed, double = _safe_parse_json_maybe_str(raw)
        if double:
            typ = f"{typ}->json"
    n = len(parsed.get("devices", [])) if isinstance(parsed, dict) else 0
    return jsonify(ok=True, key=REDIS_KEYS["udp_devices_last"], type=typ, devices=n, parsed_is_dict=isinstance(parsed, dict))

@app.get("/udp/devices/debug")
def udp_devices_debug():
    """
    Devuelve devices según la fuente solicitada por ?mode=
      mode=radar | lan | trix | mix  (default: mix)
    Útil para aislar por fuente.
    """
    mode = request.args.get("mode","mix").lower()

    # --- RADAR (bridge 172.18.*)
    base_radar = udp_radar.get_snapshot() or {}
    radar_devs = list(base_radar.get("devices", []))

    # --- LAN (Redis del sensor)
    base_redis = rget_json(REDIS_KEYS["udp_devices_last"], {}) or {}
    if isinstance(base_redis, str):  # tolera doble JSON
        try:
            base_redis = json.loads(base_redis)
        except Exception:
            base_redis = {}
    lan_devs = list(base_redis.get("devices", []))

    # --- TRIX (Docker SDK)
    trix = _list_trix_net_containers()

    if mode == "radar":
        devs = radar_devs
    elif mode == "lan":
        devs = lan_devs
    elif mode == "trix":
        devs = trix
    else:
        devs = _dedup_devices(radar_devs + lan_devs + trix)

    _tag_origin_and_esp32(devs)
    return jsonify({
        "mode": mode,
        "counts": {"radar": len(radar_devs), "lan": len(lan_devs), "trix": len(trix), "returned": len(devs)},
        "devices": devs
    })

@app.get("/udp/devices/snapshot")
def udp_devices_snapshot_breakdown():
    """
    Mezcla final + breakdown de conteos por origen.
    Es igual a /udp/devices pero con más detalles de diagnóstico.
    """
    # RADAR
    base_radar = udp_radar.get_snapshot() or {}
    radar_devs = list(base_radar.get("devices", []))

    # LAN
    base_redis = rget_json(REDIS_KEYS["udp_devices_last"], {}) or {}
    if isinstance(base_redis, str):
        try:
            base_redis = json.loads(base_redis)
        except Exception:
            base_redis = {}
    lan_devs = list(base_redis.get("devices", []))

    # TRIX
    trix = _list_trix_net_containers()

    # MIX
    mixed = _dedup_devices(radar_devs + lan_devs + trix)
    _tag_origin_and_esp32(mixed)

    return jsonify({
        "counts": {"radar": len(radar_devs), "lan": len(lan_devs), "trix": len(trix), "mix": len(mixed)},
        "seq": base_radar.get("seq"),
        "ts": base_radar.get("ts"),
        "devices": mixed
    })


# =========================
# ===== WebSocket Hub =====
# =========================
_ws_clients = {}
_ws_lock = threading.Lock()
_ws_out_q = Queue(maxsize=1000)

def _ws_enqueue_for_all(payload: dict):
    with _ws_lock:
        dead = []
        for ws, q in _ws_clients.items():
            try:
                q.put_nowait(payload)
            except Full:
                try:
                    _ = q.get_nowait()
                    q.put_nowait(payload)
                except Exception:
                    dead.append(ws)
        for ws in dead:
            _ws_clients.pop(ws, None)

def ws_broadcast(event_type: str, data: dict):
    try:
        _ws_out_q.put_nowait({"type": event_type, "data": data, "ts": time.time()})
    except Full:
        try:
            _ = _ws_out_q.get_nowait()
            _ws_out_q.put_nowait({"type": event_type, "data": data, "ts": time.time()})
        except Exception:
            pass

def _ws_broadcast_loop():
    app.logger.info("[WS] Broadcast loop iniciado", extra={"subsystem":"WS"})
    while True:
        try:
            payload = _ws_out_q.get()
            if payload is None:
                continue
            _ws_enqueue_for_all(payload)
        except Exception:
            app.logger.exception("[WS] Broadcast loop error", extra={"subsystem":"WS"})

def _ws_udp_snapshot_loop(interval=2.0):
    """Refleja LAN+trix-net+radar hacia WS periódicamente (mezcla por defecto).
       OJO: NO escribimos en la clave del sensor de Redis para no pisarla."""
    app.logger.info("[WS] UDP snapshot mirror loop iniciado (%.1fs)", interval, extra={"subsystem":"WS"})
    while True:
        try:
            devs = combined_devices_snapshot()
            ws_broadcast("udp_devices", devs)
            # (No tocar REDIS_KEYS["udp_devices_last"] aquí)
        except Exception:
            app.logger.exception("[WS] UDP snapshot mirror error", extra={"subsystem":"WS"})
        time.sleep(interval)

def _start_ws_aux():
    threading.Thread(target=_ws_broadcast_loop, daemon=True).start()
    threading.Thread(target=_ws_udp_snapshot_loop, daemon=True).start()

def _ws_reader(ws):
    while True:
        try:
            msg = ws.receive()
            if msg is None:
                break
            try:
                obj = json.loads(msg)
            except Exception:
                ws.send(json.dumps({"type":"error","message":"invalid_json"}))
                continue

            action = str(obj.get("action","")).lower()

            if action == "ping":
                ws.send(json.dumps({"type":"pong","ts": time.time()}))

            elif action == "get":
                what = str(obj.get("what","")).lower()
                if what == "snapshot":
                    s = snapshot() or rget_json(REDIS_KEYS["snapshot_last"], {}) or {}
                    ws.send(json.dumps({"type":"snapshot","data": s}))
                elif what == "modbus":
                    last = rget_json(REDIS_KEYS["modbus_last"], {}) or {}
                    ws.send(json.dumps({"type":"modbus","data": last or safe_get_modbus({})}))
                elif what == "devices":
                    devs = combined_devices_snapshot()  # mezcla por defecto
                    ws.send(json.dumps({"type":"udp_devices","data": devs}))
                else:
                    ws.send(json.dumps({"type":"error","message":"unknown_get"}))

            elif action == "setpoint":
                s = snapshot() or rget_json(REDIS_KEYS["snapshot_last"], {}) or {}
                deg_now = float(s.get("degrees", 0.0))
                if "value" in obj:
                    new_sp = float(obj["value"])
                else:
                    delta = float(obj.get("delta", 0.0))
                    new_sp = deg_now + delta
                burst = int(obj.get("burst", 1))
                ros_publish_q.put(("setpoint", {"value": new_sp, "burst": burst}))
                ws.send(json.dumps({"type":"ack","action":"setpoint","current":deg_now,"new_setpoint":new_sp,"burst":burst}))

            elif action == "toggle":
                idx = int(obj.get("index", -1))
                if idx not in (0,1,2):
                    ws.send(json.dumps({"type":"error","message":"invalid_index"}))
                    continue
                r = safe_toggle(idx)
                if r.get("status") == "success":
                    bus_fanout({"lw"+str(idx): r["new_value"]}, src="WS")
                ws.send(json.dumps({"type":"ack","action":"toggle","index":idx,"result":r}))

            else:
                ws.send(json.dumps({"type":"error","message":"unknown_action"}))

        except Exception:
            break

@sock.route("/ws")
def ws_endpoint(ws):
    per_client_q = Queue(maxsize=200)
    with _ws_lock:
        _ws_clients[ws] = per_client_q

    try:
        hello = {
            "type": "hello",
            "ts": time.time(),
            "snapshot": snapshot() or rget_json(REDIS_KEYS["snapshot_last"], {}) or {},
            "modbus": rget_json(REDIS_KEYS["modbus_last"], {}) or safe_get_modbus({}),
            "devices": combined_devices_snapshot(),  # mezcla por defecto
        }
        per_client_q.put_nowait(hello)
    except Exception:
        pass

    t_reader = threading.Thread(target=_ws_reader, args=(ws,), daemon=True)
    t_reader.start()

    try:
        while True:
            payload = per_client_q.get()
            if payload is None:
                continue
            try:
                ws.send(json.dumps(payload))
            except Exception:
                break
    finally:
        with _ws_lock:
            _ws_clients.pop(ws, None)

# =========== Bus fan-out unificado ===========
def bus_fanout(update: dict, src: str = None):
    try:
        upd = dict(update or {})
        if src:
            upd["_src"] = src
        publish_update(upd)

        try:
            s = snapshot() or {}
            if s:
                rset_json(REDIS_KEYS["snapshot_last"], s, ex=15)
        except Exception:
            pass

        if src in ("UDP", "ROS"):
            try:
                udp_radar.note_from_update(upd)
            except Exception:
                app.logger.exception("[CHAIN] publish_update->radar falló", extra={"subsystem":"UDP"})

        ws_broadcast("update", upd)
        rxadd(REDIS_KEYS["updates_stream"], {"ts": time.time(), "upd": upd}, maxlen=2000)
        rpublish(REDIS_KEYS["updates_chan"], {"ts": time.time(), "upd": upd})

    except Exception:
        app.logger.exception("[BUS] fanout error", extra={"subsystem":"WS"})

# -------- Rutas HTTP --------
@app.route("/")
def index():
    snap = snapshot() or rget_json(REDIS_KEYS["snapshot_last"], {}) or {}
    data = safe_get_modbus(default={"lw0": False, "lw1": False, "lw2": False, "hmi_online": False})
    ctx = dict(data); ctx.update({"degrees": snap.get("degrees",0.0), "pulses": snap.get("pulses",0)})
    app.logger.info("[INDEX] render keys: data=%s, snap=%s", list(data.keys()), list(snap.keys()), extra={"subsystem":"HTTP"})
    return render_template("index.html", **ctx)

@app.route("/telemetry")
def telemetry():
    s = snapshot() or rget_json(REDIS_KEYS["snapshot_last"], {}) or {}
    return jsonify({"degrees": s.get("degrees",0.0), "pulses": s.get("pulses",0)})

@app.route("/orientation")
def orientation_stream():
    app.logger.info("[SSE] client conectado a /orientation", extra={"subsystem":"SSE"})
    return Response(
        sse_stream(heartbeat_every=2.0),
        content_type="text/event-stream; charset=utf-8",
        headers={"Cache-Control":"no-cache","X-Accel-Buffering":"no","Connection":"keep-alive"},
        direct_passthrough=True
    )

@app.get("/ros/topics")
def ros_topics():
    try:
        if not rclpy.ok():
            rclpy.init(args=None)
        node = rclpy.create_node('ui_topics_probe')
        topics = node.get_topic_names_and_types()
        node.destroy_node()
        return jsonify({"ok": True, "topics": topics, "domain": os.getenv("ROS_DOMAIN_ID","?"),
                        "rmw": os.getenv("RMW_IMPLEMENTATION","?")})
    except Exception as e:
        return jsonify({"ok": False, "error": str(e)}), 500

# ===== Modbus seguros + cache =====
def safe_get_modbus(default=None):
    try:
        data = get_modbus_data()
        if not data:
            data = default or {}
        rset_json(REDIS_KEYS["modbus_last"], data, ex=10)
        return data
    except Exception as e:
        app.logger.warning("[MODBUS] get_modbus_data falló: %s", e, extra={"subsystem":"MODBUS"})
        cached = rget_json(REDIS_KEYS["modbus_last"])
        if cached:
            cached["_cached"] = True
            return cached
        return (default or {"hmi_online": False})

def safe_toggle(idx: int):
    try:
        result = toggle_lw(idx)
        if isinstance(result, dict) and result.get("status") == "success":
            current = rget_json(REDIS_KEYS["modbus_last"], {}) or {}
            current[f"lw{idx}"] = result.get("new_value")
            rset_json(REDIS_KEYS["modbus_last"], current, ex=10)
        return result
    except Exception as e:
        app.logger.warning("[MODBUS] toggle_lw(%d) falló: %s", idx, e, extra={"subsystem":"MODBUS"})
        return {"status": "error", "message": "HMI no disponible", "index": idx, "exception": str(e)}

# ====== Descubrimiento de contenedores y snapshot combinado ======
DOCKER_NETWORK = os.getenv("DOCKER_NETWORK", "trix-net")

def _resolve_docker_network(client, wanted: str):
    try:
        return client.networks.get(wanted)
    except Exception:
        pass
    nets = client.networks.list()
    for n in nets:
        try:
            if n.name.endswith(wanted):
                return n
        except Exception:
            pass
    for n in nets:
        try:
            if wanted in n.name:
                return n
        except Exception:
            pass
    return None

def _list_trix_net_containers() -> List[Dict]:
    containers = []
    try:
        import docker
        client = docker.from_env()
        net = _resolve_docker_network(client, DOCKER_NETWORK)
        if not net:
            app.logger.warning("[DOCKER] red '%s' no encontrada; disponibles=%s",
                               DOCKER_NETWORK, [n.name for n in client.networks.list()],
                               extra={"subsystem":"DOCKER"})
            return []
        now = int(time.time())
        cons = net.attrs.get("Containers") or {}
        for cid, info in cons.items():
            name = info.get("Name")
            ipv4 = (info.get("IPv4Address") or "").split("/")[0]
            mac  = info.get("MacAddress")
            containers.append({
                "ip": ipv4,
                "mac": mac,
                "name": name,
                "hostname": name,
                "vendor": None,
                "node": "",
                "topic": "",
                "uros_agent_host": "",
                "uros_agent_port": None,
                "online": True,
                "first_seen_ever": None,
                "last_seen": now,
                "sensor": "trix-net",
                "mac_last6": (mac or "")[-6:],
                "_src_net": "trix-net",
            })
        app.logger.info("[DOCKER] red usada='%s' contenedores_en_red=%d",
                        getattr(net, "name", "?"), len(containers),
                        extra={"subsystem":"DOCKER"})
        return containers
    except Exception as e:
        app.logger.warning("[DOCKER] listado de contenedores falló: %s", e, extra={"subsystem":"DOCKER"})
        return []

def _dedup_devices(devs: List[Dict]) -> List[Dict]:
    """Dedup por (mac, ip) preservando el primero."""
    seen = set(); out = []
    for d in devs or []:
        key = (d.get("mac") or "", d.get("ip") or "")
        if key in seen:
            continue
        seen.add(key); out.append(d)
    return out

def _tag_origin_and_esp32(devices: List[Dict]) -> None:
    for d in devices:
        ip = (d.get("ip") or "")
        # origen por IP si falta
        if ip.startswith("172.18."):
            d["_src_net"] = "trix-net"
        elif ip.startswith("192.168."):
            d["_src_net"] = "lan"
        else:
            d.setdefault("_src_net", d.get("sensor") or "lan")
        # marcar ESP32
        vendor = (d.get("vendor") or "").lower()
        if vendor.startswith("espressif") or ip.startswith("192.168."):
            d["_is_esp32"] = True

def combined_devices_snapshot() -> Dict:
    """
    Mezcla SIEMPRE:
      - radar interno (bridge Docker)   -> 172.18.x.x
      - LAN desde Redis                 -> 192.168.x.x
      - contenedores por Docker SDK     -> 172.18.x.x
    """
    base_radar = udp_radar.get_snapshot() or {}
    radar_devs = list(base_radar.get("devices", []))
    seq = base_radar.get("seq")
    ts  = base_radar.get("ts")

    base_redis = rget_json(REDIS_KEYS["udp_devices_last"], {}) or {}
    lan_devs = list(base_redis.get("devices", []))

    trix = _list_trix_net_containers()

    devices = _dedup_devices(radar_devs + lan_devs + trix)
    _tag_origin_and_esp32(devices)

    app.logger.info("[DEVICES] radar=%d lan=%d trix=%d total=%d",
                    len(radar_devs), len(lan_devs), len(trix), len(devices),
                    extra={"subsystem":"HTTP"})

    return {"devices": devices, "count": len(devices), "seq": seq, "ts": ts}

@app.get("/udp/devices")
def udp_devices_json():
    return jsonify(combined_devices_snapshot())

@app.get("/udp/devices/stream")
def udp_devices_stream():
    app.logger.info("[SSE] client conectado a /udp/devices/stream", extra={"subsystem":"SSE"})
    # Stream del radar. Si quieres, puedes cambiar a emitir el mix aquí.
    return Response(
        udp_radar.sse_stream(interval=2.0),
        content_type="text/event-stream; charset=utf-8",
        headers={"Cache-Control":"no-cache","X-Accel-Buffering":"no","Connection":"keep-alive"},
        direct_passthrough=True
    )

# ----- Utilidades Redis -----
@app.get("/redis/health")
def redis_health():
    r = _redis_connect()
    ok = False
    info = {}
    try:
        if r:
            ok = bool(r.ping())
            info = {"host": REDIS_HOST, "port": REDIS_PORT, "db": REDIS_DB}
    except Exception as e:
        info = {"error": str(e)}
    return jsonify(ok=ok, info=info), (200 if ok else 503)

@app.post("/redis/cache/clear")
def redis_cache_clear():
    r = _redis_connect()
    if not r:
        return jsonify(ok=False, message="Redis no disponible"), 503
    removed = 0
    for k in (REDIS_KEYS["modbus_last"], REDIS_KEYS["udp_devices_last"], REDIS_KEYS["snapshot_last"]):
        try:
            removed += int(bool(r.delete(k)))
        except Exception:
            pass
    return jsonify(ok=True, removed=removed)

# ----- Debug Docker networks -----
@app.get("/debug/docker-networks")
def debug_docker_networks():
    try:
        import docker
        client = docker.from_env()
        nets = [{"name": n.name, "id": n.id} for n in client.networks.list()]
        chosen = None
        net = _resolve_docker_network(client, DOCKER_NETWORK)
        if net:
            chosen = {"name": net.name, "id": net.id}
        return jsonify(ok=True, wanted=DOCKER_NETWORK, chosen=chosen, networks=nets)
    except Exception as e:
        return jsonify(ok=False, error=str(e)), 500

# ----- ESP-IDF -----
@app.post('/build_idf')
def build_idf():
    try:
        client = docker.from_env(); container = client.containers.get('robot')
        cmd = 'bash -c "source /opt/esp-idf/export.sh >/dev/null && cd /esp2024 && idf.py build"'
        exec_result = container.exec_run(cmd=cmd, workdir='/esp2024')
        out=(exec_result.output or b"").decode(errors="ignore")
        if exec_result.exit_code==0: return jsonify(status="success", message="Build completado.", output=out)
        return jsonify(status="error", message="Build falló.", output=out), 400
    except Exception as e:
        app.logger.exception("[ESP-IDF] build error", extra={"subsystem":"BUILD"})
        return jsonify(status="error", message=str(e)), 500

@app.post('/flash_esp32')
def flash_esp32():
    port = request.json.get('port','/dev/ttyUSB0') if request.is_json else '/dev/ttyUSB0'
    try:
        client = docker.from_env(); container = client.containers.get('robot')
        cmd = f'bash -c "source /opt/esp-idf/export.sh >/dev/null && cd /esp2024 && idf.py -p {port} flash"'
        exec_result = container.exec_run(cmd=cmd, workdir='/esp2024')
        out=(exec_result.output or b"").decode(errors="ignore")
        if exec_result.exit_code==0: return jsonify(status="success", message=f"Flash OK en {port}.", output=out)
        return jsonify(status="error", message="Flash falló.", output=out), 400
    except Exception as e:
        app.logger.exception("[ESP-IDF] flash error", extra={"subsystem":"BUILD"})
        return jsonify(status="error", message=str(e)), 500

@app.post('/idf_monitor')
def idf_monitor():
    port = request.json.get('port','/dev/ttyUSB0') if request.is_json else '/dev/ttyUSB0'
    try:
        client = docker.from_env(); container = client.containers.get('robot')
        cmd = f'bash -c "source /opt/esp-idf/export.sh >/dev/null && cd /esp2024 && idf.py -p {port} monitor"'
        exec_result = container.exec_run(cmd=cmd, workdir='/esp2024')
        out=(exec_result.output or b"").decode(errors="ignore")
        return jsonify(status="success", message=f"Monitor lanzado en {port}.", output=out)
    except Exception as e:
        app.logger.exception("[ESP-IDF] monitor error", extra={"subsystem":"BUILD"})
        return jsonify(status="error", message=str(e)), 500

@app.get("/whoami")
def whoami():
    return jsonify(app="flask", port=5500)

# ----- ROS Trigger plan -----
@app.post("/api/plan", endpoint="api_plan")
def api_plan():
    node = None
    try:
        _log_info("[/api/plan] recibido")
        if not rclpy.ok():
            rclpy.init(args=None)

        node = rclpy.create_node('ui_plan_client_once')
        client = node.create_client(Trigger, '/plan')

        if not client.wait_for_service(timeout_sec=5.0):
            app.logger.warning("Servicio /plan no disponible", extra={"subsystem":"ROS","req_id":getattr(g,'req_id',None)})
            node.destroy_node()
            return jsonify(ok=False, message="Servicio /plan no disponible"), 503

        req = Trigger.Request()
        future = client.call_async(req)

        execu = SingleThreadedExecutor()
        execu.add_node(node)
        execu.spin_until_future_complete(future, timeout_sec=10.0)

        if future.done():
            res = future.result()
            msg = getattr(res, "message", "")
            ok = bool(getattr(res, "success", False))
            _log_info("[/api/plan] respuesta", ok=ok, message=msg, subsystem="ROS")
            node.destroy_node()
            return jsonify(ok=ok, message=msg), (200 if ok else 500)

        app.logger.error("[/api/plan] timeout esperando respuesta de /plan", extra={"subsystem":"ROS","req_id":getattr(g,'req_id',None)})
        node.destroy_node()
        return jsonify(ok=False, message="Timeout al llamar /plan"), 504

    except Exception as e:
        app.logger.exception("[/api/plan] fallo", extra={"subsystem":"ROS","req_id":getattr(g,'req_id',None)})
        try:
            if node is not None:
                node.destroy_node()
        except Exception:
            pass
        return jsonify(ok=False, message=f"Exception: {e}", traceback=traceback.format_exc()), 500

# ----- Modbus / Relés -----
@app.get('/data')
def data_route():
    return jsonify(safe_get_modbus())

@app.post('/toggle_lw0')
def toggle_lw0():
    r = safe_toggle(0)
    if r.get('status')=='success':
        bus_fanout({'lw0': r['new_value']}, src="HTTP")
        ros_publish_q.put(("relay", {"index":0,"value":r['new_value']}))
    return jsonify(r)

@app.post('/toggle_lw1')
def toggle_lw1():
    r = safe_toggle(1)
    if r.get('status')=='success':
        bus_fanout({'lw1': r['new_value']}, src="HTTP")
        ros_publish_q.put(("relay", {"index":1,"value":r['new_value']}))
    return jsonify(r)

@app.post('/toggle_lw2')
def toggle_lw2():
    r = safe_toggle(2)
    if r.get('status')=='success':
        bus_fanout({'lw2': r['new_value']}, src="HTTP")
        ros_publish_q.put(("relay", {"index":2,"value":r['new_value']}))
    return jsonify(r)

# ----- Movimiento (setpoint) -----
def _move(direction:int):
    data = request.get_json(silent=True) or {}
    step  = float(request.args.get("step",  data.get("step", 1.0)))
    burst = int(request.args.get("burst", data.get("burst", 1)))
    s = snapshot() or rget_json(REDIS_KEYS["snapshot_last"], {}) or {}
    deg_now = float(s.get("degrees",0.0))
    new_sp  = deg_now + (direction*step)

    app.logger.info("[SERVO ENQ] dir=%+d step=%.2f burst=%d deg_now=%.2f -> new_sp=%.2f",
                    direction, step, burst, deg_now, new_sp, extra={"subsystem":"ROS","req_id":getattr(g,'req_id',None)})

    ros_publish_q.put(("setpoint", {"value": new_sp, "burst": burst}))
    try:
        qsz = ros_publish_q.qsize()
    except:
        qsz = "?"
    app.logger.info("[SERVO ENQ] queued to ROS: value=%.2f burst=%d qsize=%s", new_sp, burst, qsz, extra={"subsystem":"ROS","req_id":getattr(g,'req_id',None)})

    return jsonify({"status":"ok","current_degrees":deg_now,"new_setpoint":new_sp,"step":step,"burst":burst,"direction":direction})

@app.post("/servo/right")
@app.post("/api/servo/right")
def servo_right(): return _move(+10)

@app.post("/servo/left")
@app.post("/api/servo/left")
def servo_left():  return _move(-10)

# ===== BOOT: Radar/ROS/Modbus/Discovery/Redis-PSUB =====
UDP_DISC_LOG = os.environ.get("UDP_DISCOVERY_LOG", "/app/logs/udp_scan.txt")
UDP_OFFLINE_TIMEOUT = float(os.environ.get("UDP_OFFLINE_TIMEOUT_S", "12"))
UDP_FOCUS_PROBE_INTERVAL = float(os.environ.get("UDP_FOCUS_PROBE_INTERVAL_S", "3"))
UDP_PROBE_WINDOW = float(os.environ.get("UDP_PROBE_WINDOW_S", "2"))

def _create_udp_radar():
    """Crea UDPRadar de forma compatible con versiones (con/sin focus_probe_interval_s)."""
    kwargs_base = dict(
        ports_to_probe=[config.UDP_PORT, 8888],
        discovery_interval_s=2.0,
        arp_refresh_s=2.0,
        log_path=UDP_DISC_LOG,
        log_snapshot_on_sweep=False,
        offline_timeout_s=UDP_OFFLINE_TIMEOUT,
        probe_window_s=UDP_PROBE_WINDOW,
    )
    try:
        return UDPRadar(**kwargs_base, focus_probe_interval_s=UDP_FOCUS_PROBE_INTERVAL)
    except TypeError:
        radar = UDPRadar(**kwargs_base)
        try:
            if hasattr(radar, "focus_probe_interval_s"):
                setattr(radar, "focus_probe_interval_s", UDP_FOCUS_PROBE_INTERVAL)
        except Exception:
            pass
        return radar

udp_radar = _create_udp_radar()

def _start_udp():
    def chained(update: dict):
        bus_fanout(update, src="UDP")
    try:
        start_udp_listener(config.UDP_PORT, chained)
        app.logger.info("[BOOT] UDP listener OK (chained to radar + WS)", extra={"subsystem":"UDP"})
    except Exception:
        app.logger.exception("[BOOT] UDP FAILED", extra={"subsystem":"UDP"})

def _start_ros():
    def _w():
        app.logger.info("[BOOT] ROS thread starting (domain=%s)", config.ROS_DOMAIN_ID, extra={"subsystem":"ROS"})
        try:
            def chained(update: dict):
                bus_fanout(update, src="ROS")
            ros_thread(chained, config.ROS_DOMAIN_ID, ros_publish_q)
        except Exception:
            app.logger.exception("[BOOT] ROS thread crashed", extra={"subsystem":"ROS"})
        else:
            app.logger.info("[BOOT] ROS thread exited", extra={"subsystem":"ROS"})
    threading.Thread(target=_w, daemon=True).start()

def _start_modbus():
    try:
        def chained_modbus(update: dict):
            bus_fanout(update, src="MODBUS")
        start_modbus_poller(get_modbus_data, chained_modbus, interval=1.0)
        app.logger.info("[BOOT] Modbus poller OK (fanout->WS)", extra={"subsystem":"MODBUS"})
    except Exception:
        app.logger.exception("[BOOT] Modbus FAILED", extra={"subsystem":"MODBUS"})

def _start_discovery():
    try:
        udp_radar.start()
        app.logger.info("[BOOT] UDP discovery/radar OK (subnet %s)", str(getattr(udp_radar, "_cidr", "?")), extra={"subsystem":"UDP"})
    except Exception:
        app.logger.exception("[BOOT] UDP discovery FAILED", extra={"subsystem":"UDP"})

def _start_redis_psub():
    r = _redis_connect()
    if not r:
        return
    def _loop():
        try:
            pubsub = r.pubsub(ignore_subscribe_messages=True)
            pubsub.subscribe(REDIS_KEYS["updates_chan"])
            app.logger.info("[REDIS] suscrito a %s", REDIS_KEYS["updates_chan"], extra={"subsystem":"REDIS"})
            for msg in pubsub.listen():
                if not msg or msg.get("type") != "message":
                    continue
                try:
                    data = json.loads(msg.get("data","{}"))
                    upd = data.get("upd", {})
                    ws_broadcast("update", upd)
                except Exception:
                    app.logger.exception("[REDIS] pubsub parse falló", extra={"subsystem":"REDIS"})
        except Exception:
            app.logger.exception("[REDIS] pubsub loop falló", extra={"subsystem":"REDIS"})
    threading.Thread(target=_loop, daemon=True).start()

# Orden de arranque
_start_ws_aux()
_start_udp()
_start_ros()
_start_modbus()
_start_discovery()
_start_redis_psub()

if __name__ == "__main__":
    os.environ.setdefault("PYTHONUNBUFFERED","1")
    app.logger.info("[BOOT] Flask DEV 0.0.0.0:5500", extra={"subsystem":"HTTP"})
    app.run(host="0.0.0.0", port=5500, threaded=True)

