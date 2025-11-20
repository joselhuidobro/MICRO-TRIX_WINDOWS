# app/blueprints/netinfo.py
import time, subprocess
import redis, os
from flask import Blueprint, jsonify

bp = Blueprint("netinfo", __name__)
r = redis.Redis(host=os.getenv("REDIS_HOST", "redis"), port=6379, db=0)


def _safe(cmd):
    try:
        return subprocess.check_output(cmd, shell=True, timeout=1.5).decode().strip()
    except Exception:
        return ""

def build_net_info():
    public4 = _safe('curl -s https://api.ipify.org ')
    public6_raw = _safe('curl -s https://api64.ipify.org ')
    public6 = public6_raw if ":" in public6_raw else None
    lan_ip   = _safe("ip route get 1 | awk '{for(i=1;i<=NF;i++) if($i==\"src\") print $(i+1)}'")
    docker_eth0 = _safe("ip -4 addr show eth0 | awk '/inet /{print $2}' | cut -d/ -f1")
    wg_ip   = _safe("ip -4 addr show wg0 | awk '/inet /{print $2}' | cut -d/ -f1")
    lan_list = [ip for ip in [lan_ip] if ip]
    return {
        "public_ipv4": public4 or None,
        "public_ipv6": public6,
        "wg": {"server_ip": wg_ip or None, "endpoint": None},
        "lan": {"ipv4": lan_list},
        "docker": {"bridge_ipv4": docker_eth0 or None},
        "ts": int(time.time()),
        "admin": False,
    }

@bp.get("/net/info")
def net_info():
    try:
        return jsonify(build_net_info()), 200, {"Cache-Control":"no-store"}
    except Exception as e:
        return jsonify({"status":"error","message":str(e)}), 503, {"Cache-Control":"no-store"}

@bp.get("/net/public-ip")
def public_ip():
    return jsonify({
        "ipv4": _safe('curl -s https://api.ipify.org ') or None,
        "ipv6": _safe('curl -s https://api64.ipify.org ') or None,
        "ts": int(time.time())
    }), 200, {"Cache-Control":"no-store"}