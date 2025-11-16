# rt_bus.py
import time
import threading
from collections import deque

try:
    # usa tu súper rápido si existe
    from json_utils import fast_dumps as dumps
except Exception:
    import json
    dumps = json.dumps

# ===== Estado compartido =====
# rt_bus.py  (solo el dict 'latest' cambia)
latest = {
    "pitch": 0.0, "roll": 0.0, "yaw": 0.0,

    # <<< AÑADE ESTO >>>
    "degrees": 0.0,
    "pulses": 0,

    "m1": 0, "m2": 0, "m3": 0, "m4": 0,
    "t_us": None,
    "lat_ms": None,
    "lw0": False, "lw1": False, "lw2": False,
}

_lock_latest = threading.Lock()

# ===== Señalización para SSE =====
_updates_q = deque(maxlen=256)
_updates_evt = threading.Event()

def snapshot() -> dict:
    """Copia atómica del estado para plantillas / respuestas iniciales."""
    with _lock_latest:
        return dict(latest)

def publish_update(payload: dict):
    """Actualiza estado y despierta a los consumidores SSE."""
    if not isinstance(payload, dict):
        return
    with _lock_latest:
        latest.update(payload)
        snap = dict(payload)   # lo que empujaremos a SSE
    # logging sencillo en terminal
    print("[RT] update:", snap)
    _updates_q.append(snap)
    _updates_evt.set()

def _sse_bytes(payload: dict) -> bytes:
    return ("data:" + dumps(payload) + "\n\n").encode("utf-8")

def sse_stream(heartbeat_every: float = 2.0):
    """
    Generador para `Response(..., content_type='text/event-stream')`.
    Envía un snapshot inicial y luego solo el último cambio agregado,
    con heartbeats periódicos.
    """
    # primer envío: snapshot completo
    yield _sse_bytes(snapshot())

    last_sent = time.time()
    while True:
        fired = _updates_evt.wait(timeout=heartbeat_every)
        if fired:
            _updates_evt.clear()
            last = None
            # drena la cola y manda solo el último (backpressure friendly)
            while _updates_q:
                last = _updates_q.pop()
                _updates_q.clear()
            if last is not None:
                yield _sse_bytes(last)
                last_sent = time.time()
        else:
            # heartbeat
            yield b":\n\n"
            last_sent = time.time()

