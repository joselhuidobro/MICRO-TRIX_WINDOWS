# modbus_service.py  (reemplaza el contenido que te di por este)
import time
import threading

def _normalize(cur):
    """Convierte la salida de get_modbus_data en payload listo para UI/SSE."""
    if not isinstance(cur, dict):
        return {"modbus": cur}

    payload = dict(cur)  # conserva lo original (p.ej., 'Degrees')
    # Añade alias en snake_case usados por el front
    if "Degrees" in cur and "degrees" not in cur:
        payload["degrees"] = cur["Degrees"]
    if "Pulses" in cur and "pulses" not in cur:
        payload["pulses"] = cur["Pulses"]
    return payload

def _modbus_loop(get_modbus_data, publish_update, interval: float = 1.0):
    prev = object()  # marcador de "no hay previo"
    interval = max(0.05, float(interval))
    while True:
        try:
            cur = get_modbus_data()
            payload = _normalize(cur)
            if payload != prev:
                publish_update(payload)
                prev = payload
        except Exception as e:
            print(f"[Modbus] Error polling: {e}")
        time.sleep(interval)

def start_modbus_poller(get_modbus_data, publish_update, interval: float = 1.0):
    th = threading.Thread(
        target=_modbus_loop,
        args=(get_modbus_data, publish_update, interval),
        daemon=True,
    )
    th.start()
    return th

