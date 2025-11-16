# udp_discovery.py
import os
import re
import json
import time
import socket
import ipaddress
import threading
import subprocess
from typing import Dict, Any, List, Tuple, Optional

# --- OUIs comunes de Espressif (ampliado) ---
ESPRESSIF_OUIS = {
    "24:6f:28", "7c:df:a1", "30:ae:a4", "b4:e6:2d", "84:f7:03",
    "3c:61:05", "b8:f0:09", "bc:dd:c2", "f4:12:fa", "ac:67:b2",
    "34:5f:45"  # módulos Wi-Fi usados con ESP*
}

_LAST6_RE = re.compile(r'([0-9A-Fa-f]{6})(?:\b|$)')


def _now_ts() -> float:
    return time.time()


def _ts_iso() -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime()) + f".{int((time.time()%1)*1000):03d}Z"


def _iso_from(ts: float) -> str:
    return time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(ts)) + f".{int((ts%1)*1000):03d}Z"


def _mac_vendor_guess(mac: str) -> str:
    if not mac:
        return ""
    oui = mac.lower().strip().replace('-', ':')[:8]
    if oui in ESPRESSIF_OUIS:
        return "Espressif"
    if oui.startswith("ac:67:b2") or oui.startswith("8c:aa:b5"):
        return "Espressif (newer)"
    return ""


def _read_arp_table(only_complete: bool = True) -> List[Dict[str, str]]:
    """
    Lee /proc/net/arp. Cuando only_complete=True, filtra por flags=0x2.
    Devuelve dicts con ip, mac, iface, flags (lower).
    """
    out: List[Dict[str, str]] = []
    try:
        with open("/proc/net/arp", "r") as f:
            lines = f.read().strip().splitlines()
        for ln in lines[1:]:
            parts = ln.split()
            if len(parts) >= 6:
                ip, hw_type, flags, mac, mask, iface = parts[:6]
                flags = flags.lower()
                mac = mac.lower()
                if only_complete and flags != "0x2":
                    continue
                out.append({"ip": ip, "mac": mac, "iface": iface, "flags": flags})
    except Exception:
        pass
    return out


def _reverse_dns(ip: str) -> str:
    try:
        name, _, _ = socket.gethostbyaddr(ip)
        return name
    except Exception:
        return ""


def _default_local_ip() -> str:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        return s.getsockname()[0]
    except Exception:
        return "127.0.0.1"
    finally:
        try: s.close()
        except Exception: pass


def _cidr24_for_ip(ip: str) -> ipaddress.IPv4Network:
    try:
        return ipaddress.ip_network(ip + "/24", strict=False)
    except Exception:
        return ipaddress.ip_network("192.168.0.0/24", strict=False)


def _norm_last6(x: str) -> str:
    if not x:
        return ""
    x = x.strip().replace(":", "").replace("-", "")
    if len(x) == 6 and all(c in "0123456789abcdefABCDEF" for c in x):
        return x.lower()
    m = _LAST6_RE.search(x)
    return m.group(1).lower() if m else ""


def _mac_to_last6(mac: str) -> str:
    mac = (mac or "").lower().replace("-", ":")
    parts = mac.split(":")
    return "".join(parts[-3:]) if len(parts) == 6 else ""


def _parse_uros_agent(agent_str: Any) -> Tuple[str, int]:
    """
    Acepta "192.168.0.203:9999" o dict {"host":..., "port":...}
    """
    if isinstance(agent_str, dict):
        host = str(agent_str.get("host", "") or "")
        port = int(agent_str.get("port", 0) or 0)
        return host, port
    if not agent_str:
        return "", 0
    s = str(agent_str)
    if ":" in s:
        h, p = s.rsplit(":", 1)
        try: p = int(p)
        except Exception: p = 0
        return h, p
    return s, 0


class UDPRadar:
    """
    Escaneo /24 + ARP + updates (UDP/ROS). REGISTRA SOLO TRANSICIONES:
      - [UP] primera vez online (por sesión)
      - [DOWN] al perderlo (timeout o verificación rápida)
    Mejoras clave para cortes rápidos:
      - Probing focalizado a ONLINE cada `focus_probe_interval_s`
      - `last_seen` SOLO se actualiza si hubo probe reciente (ventana `probe_window_s`)
        o si llegó un update real (note_from_update/note_from_udp)
      - Verificación opcional vía `ip neigh` para bajar antes del timeout
    """

    def __init__(
        self,
        ports_to_probe: List[int],
        discovery_interval_s: float = 30.0,
        arp_refresh_s: float = 2.0,
        iface_ip: Optional[str] = None,
        log_path: str = "udp_discovery.log",
        log_snapshot_on_sweep: bool = False,
        offline_timeout_s: float = 8.0,          # <— por defecto 12 s
        focus_probe_interval_s: float = 2.0,      # probear ONLINE cada 3 s
        probe_window_s: float = 1.5,              # ventana para aceptar ARP como "visto"
    ):
        self._ports = list({p for p in ports_to_probe if isinstance(p, int) and p > 0})
        self._discovery_interval = max(5.0, float(discovery_interval_s))
        self._arp_refresh = max(1.0, float(arp_refresh_s))
        self._offline_timeout = max(3.0, float(offline_timeout_s))

        self._focus_probe_interval = max(0.5, float(focus_probe_interval_s))
        self._probe_window = max(0.3, float(probe_window_s))

        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._devices: Dict[str, Dict[str, Any]] = {}  # key = ip
        self._seq = 0

        self._iface_ip = iface_ip or _default_local_ip()
        self._cidr = _cidr24_for_ip(self._iface_ip)

        # control de probes recientes para validar ARP "fresco"
        self._recent_probe: Dict[str, float] = {}

        # Logging a archivo
        self._log_path = log_path
        self._log_snapshot_on_sweep = bool(log_snapshot_on_sweep)
        self._log_lock = threading.Lock()
        self._ensure_log_dir()
        self._append_log(f"{_ts_iso()} [BOOT] UDPRadar up. iface_ip={self._iface_ip} subnet={self._cidr}")

        # Hilos
        self._t_arp    = threading.Thread(target=self._arp_watcher_loop, daemon=True)
        self._t_sweep  = threading.Thread(target=self._sweep_loop, daemon=True)
        self._t_state  = threading.Thread(target=self._state_loop, daemon=True)
        self._t_focus  = threading.Thread(target=self._focus_probe_loop, daemon=True)

    # ---------- API pública ----------
    def start(self):
        self._t_arp.start()
        self._t_sweep.start()
        self._t_state.start()
        self._t_focus.start()

    def stop(self):
        self._stop.set()

    def note_from_udp(self, ip: str = "", name: str = "", mac: str = "", extra: Optional[Dict[str, Any]] = None):
        if not ip and extra and "ip" in extra:
            ip = str(extra["ip"])
        if not ip:
            return
        fields: Dict[str, Any] = {}
        if name: fields["name"] = name
        if mac:  fields["mac"]  = mac.lower()
        if extra:
            fields["tags"] = extra.get("tags")
            for k in ("node","node_name","Nodo","nodo","topic","Tópico","topico","Topic",
                      "uros_agent","uROS_agent","agent","host","mac","hwaddr","ip","id"):
                if k in extra: fields[k] = extra[k]
        self._merge(ip, fields, src="UDP")

    def note_from_update(self, update: Dict[str, Any]):
        if not isinstance(update, dict):
            return
        src = (update.get("_src") or update.get("src") or "UDP")
        ip = update.get("ip") or update.get("host") or ""
        if not ip:
            return

        fields: Dict[str, Any] = {}
        mac = (update.get("mac") or update.get("hwaddr") or "").lower()
        if mac: fields["mac"] = mac

        node  = (update.get("node") or update.get("node_name") or update.get("Nodo") or update.get("nodo") or "")
        topic = (update.get("topic") or update.get("Tópico") or update.get("topico") or update.get("Topic") or "")
        if node:  fields["node"]  = node
        if topic: fields["topic"] = topic

        agent = (update.get("uros_agent") or update.get("uROS_agent") or update.get("agent") or "")
        a_host, a_port = _parse_uros_agent(agent)
        if a_host: fields["uros_agent_host"] = a_host
        if a_port: fields["uros_agent_port"] = a_port

        last6 = _norm_last6(node) or _norm_last6(topic) or _norm_last6(update.get("id", ""))
        if last6: fields["mac_last6"] = last6

        t = update.get("tags")
        if isinstance(t, (list, set, tuple)): fields["tags"] = t

        self._merge(ip, fields, src=src)

    def get_snapshot(self) -> Dict[str, Any]:
        with self._lock:
            rows: List[Dict[str, Any]] = []
            for ip, d in self._devices.items():
                row = dict(d)
                if isinstance(row.get("tags"), set):
                    row["tags"] = sorted(list(row["tags"]))
                rows.append(row)
            rows.sort(key=lambda x: x.get("last_seen", 0), reverse=True)
            return {"devices": rows, "count": len(rows), "seq": self._seq, "ts": _now_ts()}

    def sse_stream(self, interval: float = 2.0):
        last_seq = -1
        while not self._stop.is_set():
            snap = self.get_snapshot()
            if snap["seq"] != last_seq:
                last_seq = snap["seq"]
                yield "event: devices\n"
                yield "data: " + json.dumps(snap, ensure_ascii=False) + "\n\n"
            else:
                yield "event: ping\n"
                yield "data: {}\n\n"
            time.sleep(max(0.5, float(interval)))

    # ---------- Internos ----------
    def _arp_watcher_loop(self):
        while not self._stop.is_set():
            try:
                entries = _read_arp_table(only_complete=True)
                now = _now_ts()
                with self._lock:
                    for e in entries:
                        ip = e.get("ip")
                        mac = e.get("mac", "")
                        if not ip or not mac or mac == "00:00:00:00:00:00":
                            continue

                        # SOLO aceptamos ARP como "visto" si hubo probe reciente
                        probe_ts = self._recent_probe.get(ip, 0.0)
                        if not probe_ts or (now - probe_ts) > self._probe_window:
                            continue

                        d = self._devices.get(ip, {"ip": ip})
                        if d.get("mac") != mac:
                            d["mac"] = mac
                            vend = _mac_vendor_guess(mac)
                            if vend: d["vendor"] = vend
                            d["mac_last6"] = _mac_to_last6(mac)

                        if "hostname" not in d:
                            hn = _reverse_dns(ip)
                            if hn: d["hostname"] = hn

                        d.setdefault("first_seen_ever", now)
                        d["last_seen"] = now
                        d["last_src"]  = "ARP"

                        if not d.get("online") and d.get("mac"):
                            d["online"] = True
                            d["session_start"] = now
                            self._log_transition_up(d, src="ARP")

                        self._devices[ip] = d
                    self._seq += 1
            except Exception:
                pass
            time.sleep(self._arp_refresh)

    def _sweep_loop(self):
        """
        Barrido general /24 para descubrimiento amplio (menos frecuente).
        """
        while not self._stop.is_set():
            try:
                self._iface_ip = _default_local_ip()
                self._cidr = _cidr24_for_ip(self._iface_ip)
                for host in self._cidr.hosts():
                    if self._stop.is_set(): break
                    hip = str(host)
                    if hip == self._iface_ip: continue
                    for port in (self._ports or [8888]):
                        try:
                            self._poke(hip, port)
                        except Exception:
                            pass
                    time.sleep(0.02)
            except Exception:
                pass

            if self._log_snapshot_on_sweep:
                snap = self.get_snapshot()
                self._append_log(f"{_ts_iso()} [SNAPSHOT] devices={snap['count']} subnet={self._cidr}")
                for d in snap["devices"]:
                    self._append_log(self._fmt_line(kind="SNAPSHOT", d=d, src="SWEEP"))

            time.sleep(self._discovery_interval)

    def _focus_probe_loop(self):
        """
        Probing rápido SOLO a los que están ONLINE para detectar caídas antes.
        """
        while not self._stop.is_set():
            try:
                with self._lock:
                    online_ips = [ip for ip, d in self._devices.items() if d.get("online")]
                for ip in online_ips:
                    for port in (self._ports or [8888]):
                        try:
                            self._poke(ip, port)
                        except Exception:
                            pass
                    time.sleep(0.01)  # ritmo suave
            except Exception:
                pass
            time.sleep(self._focus_probe_interval)

    def _state_loop(self):
        """
        Marca [DOWN] por timeout, y si es posible, usa ip neigh para acelerar la caída.
        """
        while not self._stop.is_set():
            now = _now_ts()
            try:
                with self._lock:
                    for ip, d in list(self._devices.items()):
                        if not d.get("online"):
                            continue
                        since = now - d.get("last_seen", 0)
                        # Fast check con ip neigh a mitad del timeout
                        if since > min(self._offline_timeout/2.0, 6.0):
                            fast = self._fast_down_check(ip)
                            if fast is True:
                                d["online"] = False
                                d["session_end"] = now
                                self._log_transition_down(d)
                                self._devices[ip] = d
                                continue
                        # Timeout normal
                        if since > self._offline_timeout:
                            d["online"] = False
                            d["session_end"] = now
                            self._log_transition_down(d)
                            self._devices[ip] = d
            except Exception:
                pass
            time.sleep(1.0)

    # ---------- helpers de probing / verificación ----------
    def _poke(self, ip: str, port: int):
        """
        Envía un pequeño datagrama UDP para forzar resolución ARP/vecino.
        Registra el probe para que ARP watcher considere 'fresh'.
        """
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            s.settimeout(0.05)
            msg = f"TRIX_DISCOVER_V1 {int(_now_ts())}".encode()
            s.sendto(msg, (ip, int(port)))
        finally:
            try: s.close()
            except Exception: pass
        # registra probe reciente
        self._recent_probe[ip] = _now_ts()

    def _fast_down_check(self, ip: str) -> Optional[bool]:
        """
        Usa 'ip neigh show <ip>' si está disponible. Si el estado es FAILED/INCOMPLETE tras
        un pequeño poke, devolvemos True (DOWN). Si no está disponible o el estado no es concluyente,
        devuelve False o None.
        """
        # pequeño poke para activar resolución
        try:
            for port in (self._ports or [8888]):
                self._poke(ip, port)
        except Exception:
            pass

        # breve espera para que el kernel intente resolver
        time.sleep(0.08)

        try:
            r = subprocess.run(
                ["ip", "-o", "neigh", "show", ip],
                capture_output=True, text=True, timeout=0.5
            )
            out = (r.stdout or "") + (r.stderr or "")
            line = out.strip().lower()
            if not line:
                return None
            # estados típicos: REACHABLE, STALE, DELAY, PROBE, FAILED, INCOMPLETE
            if "failed" in line or "incomplete" in line:
                return True
            # reachable/probe/delay => no determinante para caída
            return False
        except Exception:
            return None

    # ---------- merge y logging de transiciones ----------
    def _merge(self, ip: str, fields: Dict[str, Any], src: str):
        with self._lock:
            d = self._devices.get(ip, {"ip": ip})
            if "tags" not in d: d["tags"] = set()
            if isinstance(fields.get("tags"), (list, set, tuple)):
                d["tags"].update(fields["tags"])

            for k in ("name","node","topic","hostname","uros_agent_host","uros_agent_port","mac_last6"):
                if k in fields and fields[k]:
                    d[k] = fields[k]

            mac = (fields.get("mac") or d.get("mac") or "").lower()
            if mac:
                d["mac"] = mac
                d["vendor"] = d.get("vendor") or _mac_vendor_guess(mac)
                d["mac_last6"] = d.get("mac_last6") or _mac_to_last6(mac)

            agent = fields.get("uros_agent") or fields.get("agent")
            if agent:
                a_host, a_port = _parse_uros_agent(agent)
                if a_host: d["uros_agent_host"] = a_host
                if a_port: d["uros_agent_port"] = a_port

            now = _now_ts()
            d.setdefault("first_seen_ever", now)
            d["last_seen"] = now
            d["last_src"]  = src

            if d.get("mac_last6") and not d.get("mac"):
                full = self._infer_full_mac_from_last6(d["mac_last6"], exclude_ip=ip)
                if full:
                    d["mac"] = full
                    d["vendor"] = d.get("vendor") or _mac_vendor_guess(full)

            if not d.get("online") and d.get("mac") and d["mac"] != "00:00:00:00:00:00":
                d["online"] = True
                d["session_start"] = now
                self._log_transition_up(d, src=src)

            self._devices[ip] = d
            self._seq += 1

    # ---------- utilidades logging ----------
    def _ensure_log_dir(self):
        if not self._log_path:
            return
        folder = os.path.dirname(self._log_path)
        if folder:
            os.makedirs(folder, exist_ok=True)

    def _append_log(self, line: str):
        if not self._log_path:
            return
        try:
            with self._log_lock:
                with open(self._log_path, "a", encoding="utf-8") as f:
                    f.write(line.rstrip() + "\n")
        except Exception:
            pass

    def _fmt_line(self, kind: str, d: Dict[str, Any], src: str = "ARP", include_exit: bool = False) -> str:
        ip = d.get("ip", "")
        mac = (d.get("mac", "") or "").lower()
        vendor = d.get("vendor", "")
        host = d.get("hostname", "")
        name = d.get("name", "")
        node = d.get("node", "")
        topic = d.get("topic", "")
        last6 = d.get("mac_last6", "")
        a_host = d.get("uros_agent_host", "")
        a_port = d.get("uros_agent_port", 0)
        tags = d.get("tags", [])
        if isinstance(tags, set): tags = sorted(list(tags))
        tags_str = ",".join([str(t) for t in tags]) if isinstance(tags, (list, tuple)) else str(tags)
        entry_ts = d.get("session_start") or d.get("first_seen_ever") or d.get("last_seen", 0.0)
        line = (
            f"{_ts_iso()} [{kind}] SRC={src} ip={ip} mac={mac} mac_last6={last6} vendor={vendor} "
            f"hostname={host} name={name} node={node} topic={topic} "
            f"uros_agent={a_host}:{a_port} tags={tags_str} "
            f"entry={_iso_from(entry_ts)}"
        )
        if include_exit:
            exit_ts = d.get("session_end", _now_ts())
            duration = max(0, int(exit_ts - (entry_ts or exit_ts)))
            line += f" exit={_iso_from(exit_ts)} duration_s={duration}"
        return line

    def _log_transition_up(self, d: Dict[str, Any], src: str = "ARP"):
        mac = (d.get("mac", "") or "").lower()
        if not mac or mac == "00:00:00:00:00:00":
            return
        self._append_log(self._fmt_line(kind="UP", d=d, src=src, include_exit=False))

    def _log_transition_down(self, d: Dict[str, Any]):
        mac = (d.get("mac", "") or "").lower()
        if not mac or mac == "00:00:00:00:00:00":
            return
        self._append_log(self._fmt_line(kind="DOWN", d=d, src=d.get("last_src", "STATE"), include_exit=True))

    def _infer_full_mac_from_last6(self, last6: str, exclude_ip: str = "") -> str:
        last6 = (last6 or "").lower()
        candidates: List[str] = []
        for ip, dv in self._devices.items():
            if exclude_ip and ip == exclude_ip:
                continue
            m = dv.get("mac", "")
            if m and _mac_to_last6(m) == last6:
                candidates.append(m)
        return candidates[0] if len(candidates) == 1 else ""

