import socket
import struct
import time
from shared import _publish_update
from config import UDP_PORT

_PACK_NO_TS = struct.Struct("<3f4h")
_PACK_TS = struct.Struct("<3f4hI")
LEN_NO_TS = _PACK_NO_TS.size
LEN_TS = _PACK_TS.size
MAX_LEN = max(LEN_TS, LEN_NO_TS)

def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)
    except OSError:
        pass
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
            if n >= LEN_TS:
                pitch, roll, yaw, m1, m2, m3, m4, t_us = _PACK_TS.unpack_from(view)
            elif n >= LEN_NO_TS:
                pitch, roll, yaw, m1, m2, m3, m4 = _PACK_NO_TS.unpack_from(view)
                t_us = None
            else:
                continue
            now_us = time.time_ns() // 1_000
            lat_ms = None
            if t_us is not None:
                dt_us = (now_us - int(t_us)) & 0xFFFFFFFFFFFFFFFF
                lat_ms = round(max(0, dt_us) / 1000.0, 2)
            payload = {
                "pitch": float(pitch),
                "roll": float(roll),
                "yaw": float(yaw),
                "m1": int(m1), "m2": int(m2), "m3": int(m3), "m4": int(m4),
                "t_us": int(t_us) if t_us is not None else None,
                "lat_ms": lat_ms
            }
            _publish_update(payload)
        except struct.error:
            continue
        except Exception as e:
            print(f"[UDP] Error: {e}")
            time.sleep(0.005)
