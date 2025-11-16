# logging_setup.py
import json, logging, socket, os, time

HOST = socket.gethostname()
SVC  = os.getenv("SERVICE_NAME", "trix-ui")

class JsonFormatter(logging.Formatter):
    def format(self, record):
        base = {
            "ts": time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime()) + f".{int(record.msecs):03d}Z",
            "level": record.levelname.lower(),
            "svc": SVC,
            "host": HOST,
            "pid": record.process,
            "thread": record.threadName,
            "subsystem": getattr(record, "subsystem", None),
            "req_id": getattr(record, "req_id", None),
            "msg": record.getMessage(),
        }
        if record.exc_info:
            base["exc_info"] = self.formatException(record.exc_info)
        # Adjunta cualquier “extra” como campos
        for k, v in getattr(record, "__dict__", {}).items():
            if k not in base and not k.startswith("_") and isinstance(v, (str, int, float, bool, dict, list, type(None))):
                base[k] = v
        return json.dumps(base, ensure_ascii=False)

def setup_json_logging(level="INFO"):
    h = logging.StreamHandler()
    h.setFormatter(JsonFormatter())
    root = logging.getLogger()
    root.handlers[:] = [h]
    root.setLevel(level)

