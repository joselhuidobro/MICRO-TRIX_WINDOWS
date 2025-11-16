# utils_public_ip.py
import time, requests, json
from redis import Redis

REDIS_URL = "redis://:trixredis123@redis_trix:6379/0"
KEY = "trix:public_ip"

def get_public_ip():
    out = {"ipv4": None, "ipv6": None, "ts": int(time.time())}
    try:
        out["ipv4"] = requests.get("https://api.ipify.org", timeout=2).text.strip()
    except: pass
    try:
        out["ipv6"] = requests.get("https://api64.ipify.org", timeout=2).text.strip()
    except: pass
    return out

def refresh_and_store():
    r = Redis.from_url(REDIS_URL)
    data = get_public_ip()
    r.set(KEY, json.dumps(data), ex=3600)  # 1h TTL
    return data

