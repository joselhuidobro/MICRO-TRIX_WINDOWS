#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TRIX Net Sensor Agent
- Sniff pasivo: ARP + mDNS + ICMPv6 (ND)
- Escaneo profundo:
    * ARP sweep activo (IPv4)
    * ICMPv6 Neighbor Solicitation sweep (IPv6)
    * Mini-portscan TCP (IPv4 + IPv6, puertos comunes)
    * Sondeo SSDP (UPnP) para nombres/servicios por multicast
- Persistencia en Redis: snapshot de devices + puertos/servicios
"""

import os, time, json, threading, socket, ipaddress, fnmatch, subprocess, re
from typing import Optional, Dict, Any, List, Set, Tuple

from scapy.all import (  # type: ignore
    sniff, ARP, UDP, IP, IPv6, Ether, Raw, conf, get_if_list, srp,
    Ether as ScapyEther, ARP as ScapyARP, ICMPv6ND_NS, ICMPv6ND_NA,
    IPv6 as ScapyIPv6, ICMPv6ND_NS as ScapyICMPv6ND_NS
)
import redis  # type: ignore
import requests

# ───────────────────────── Config ─────────────────────────
DOCKER_NETS_CIDR = os.getenv("DOCKER_NETS", "172.17.0.0/16,172.18.0.0/16")
LAN_NETS_CIDR    = os.getenv("LAN_NETS", "192.168.0.0/24")  # ajusta a tu LAN
HIDE_DOCKER      = os.getenv("HIDE_DOCKER", "false").lower() in ("1","true","yes")
RESOLVE_RDNS     = os.getenv("RESOLVE_RDNS","false").lower() in ("1","true","yes")

SENSOR_NAME = os.getenv("SENSOR_NAME", "default-sensor")

REDIS_HOST  = os.getenv("REDIS_HOST", "127.0.0.1")
REDIS_PORT  = int(os.getenv("REDIS_PORT", "6379"))
REDIS_PASS  = os.getenv("REDIS_PASSWORD", "")
REDIS_DB    = int(os.getenv("REDIS_DB", "0"))
REDIS_KEY_DEVICES = os.getenv("REDIS_KEY_DEVICES", "trix:udp:devices:last")

ENV_IFACE = (os.getenv("IFACE") or "").strip()
SCAN_IFACES_ENV = (os.getenv("SCAN_IFACES") or "").strip()
SUBNETS_EXTRA_ENV = (os.getenv("SUBNETS_EXTRA") or "").strip()
PORTS_TCP_ENV = (os.getenv("PORTS_TCP") or
                 "22,80,443,1883,8883,8080,3000,6379,9100,8123,502,9200,5601,3306,5432")

SCAN_INTERVAL_ARP   = int(os.getenv("SCAN_INTERVAL_ARP", "60"))
SCAN_INTERVAL_PORTS = int(os.getenv("SCAN_INTERVAL_PORTS", "120"))
TCP_TIMEOUT         = float(os.getenv("TCP_TIMEOUT", "0.35"))
MCAST_PROBE_INTERVAL= int(os.getenv("MCAST_PROBE_INTERVAL", "90"))
ARP_MAX_HOSTS       = int(os.getenv("ARP_MAX_HOSTS", "65536"))

ESP_PREFIXES = {"3c:61:05", "24:6f:28", "7c:df:a1", "84:f7:03", "bc:dd:c2"}

# ───────── Estado ─────────
devices: Dict[str, Dict[str, Any]] = {}
ip_to_mac: Dict[str, str] = {}
lock = threading.Lock()
now = lambda: int(time.time())

# ───────────────────────── Util ─────────────────────────
def vendor_from_mac(mac: str) -> Optional[str]:
    m = (mac or "").lower()
    return "Espressif" if any(m.startswith(p) for p in ESP_PREFIXES) else None

def mac_is_valid(mac: str) -> bool:
    if not mac: return False
    m = mac.lower()
    if m in ("00:00:00:00:00:00", "ff:ff:ff:ff:ff:ff"): return False
    parts = m.split(":")
    if len(parts) != 6: return False
    return all(len(p)==2 and all(c in "0123456789abcdef" for c in p) for p in parts)

_docker_nets: List[ipaddress.IPv4Network] = []
for c in DOCKER_NETS_CIDR.split(","):
    c = c.strip()
    if c:
        try: _docker_nets.append(ipaddress.IPv4Network(c, strict=False))
        except: pass

_lan_nets: List[ipaddress.IPv4Network] = []
for c in LAN_NETS_CIDR.split(","):
    c = c.strip()
    if c:
        try: _lan_nets.append(ipaddress.IPv4Network(c, strict=False))
        except: pass

def classify_scope(ip: str) -> str:
    try:
        ip4 = ipaddress.IPv4Address(ip)
        for n in _docker_nets:
            if ip4 in n: return "DOCKER"
        for n in _lan_nets:
            if ip4 in n: return "LAN"
    except: pass
    return "HOST"

def iface_exists(name: str) -> bool:
    return bool(name) and os.path.exists(f"/sys/class/net/{name}")

def default_iface_from_proc() -> Optional[str]:
    try:
        with open("/proc/net/route") as f:
            for line in f.read().strip().splitlines()[1:]:
                cols = line.split()
                if len(cols) < 4: continue
                iface, dest_hex, flags_hex = cols[0], cols[1], cols[3]
                if dest_hex == "00000000" and (int(flags_hex, 16) & 0x2):
                    if not iface.startswith(("lo", "veth")) and iface_exists(iface):
                        return iface
    except Exception:
        pass
    return None

def default_iface_from_scapy() -> Optional[str]:
    try:
        r = conf.route.route("0.0.0.0")
        if len(r) >= 3:
            cand = r[2]
            if cand and not str(cand).startswith(("lo", "veth")) and iface_exists(cand):
                return cand
    except Exception:
        pass
    return None

def pick_iface() -> str:
    if ENV_IFACE and iface_exists(ENV_IFACE):
        return ENV_IFACE
    cand = default_iface_from_proc()
    if cand: return cand
    cand = default_iface_from_scapy()
    if cand: return cand
    try:
        for name in get_if_list():
            if not name or name.startswith(("lo", "veth")): continue
            if iface_exists(name): return name
    except Exception:
        pass
    return "any"

IFACE = pick_iface()

def parse_ports_env(s: str) -> List[int]:
    out: List[int] = []
    for p in s.split(","):
        p = p.strip()
        if not p: continue
        try: out.append(int(p))
        except: pass
    seen=set(); res=[]
    for p in out:
        if p not in seen:
            seen.add(p); res.append(p)
    return res

PORTS_TCP = parse_ports_env(PORTS_TCP_ENV)

def match_iface_patterns(name: str, patterns: List[str]) -> bool:
    if not patterns: return True
    return any(fnmatch.fnmatch(name, pat.strip()) for pat in patterns if pat.strip())

def list_candidate_ifaces() -> List[str]:
    patterns = [p for p in SCAN_IFACES_ENV.split(",") if p.strip()] if SCAN_IFACES_ENV else []
    names = []
    for n in get_if_list():
        if n.startswith(("lo", "veth")):
            continue
        if not iface_exists(n): continue
        if patterns and not match_iface_patterns(n, patterns):
            continue
        names.append(n)
    return names

def get_iface_ipv4(nic: str) -> Optional[Tuple[str, str]]:
    try:
        ip = conf.route.get_if_addr(nic)
        nm = conf.route.get_if_netmask(nic)
        if ip and nm and nm != "0.0.0.0":
            return ip, nm
    except Exception:
        pass
    try:
        out = subprocess.check_output(["ip","-4","addr","show",nic], text=True, stderr=subprocess.DEVNULL)
        m = re.search(r"inet\s+(\d+\.\d+\.\d+\.\d+)/(\d+)", out)
        if m:
            ip = m.group(1)
            prefix = int(m.group(2))
            mask_int = (0xffffffff << (32 - prefix)) & 0xffffffff
            mask = socket.inet_ntoa(mask_int.to_bytes(4, "big"))
            return ip, mask
    except Exception:
        pass
    return None

def get_iface_ipv6(nic: str) -> Optional[str]:
    try:
        out = subprocess.check_output(["ip","-6","addr","show",nic], text=True, stderr=subprocess.DEVNULL)
        for line in out.splitlines():
            m = re.search(r"inet6\s+([0-9a-fA-F:]+)/\d+", line)
            if m:
                addr = m.group(1)
                if not addr.startswith("fe80"):
                    return addr
    except Exception:
        pass
    return None

def networks_from_ifaces() -> Tuple[List[ipaddress.IPv4Network], List[ipaddress.IPv6Network]]:
    nets4: List[ipaddress.IPv4Network] = []
    nets6: List[ipaddress.IPv6Network] = []
    for nic in list_candidate_ifaces():
        v4 = get_iface_ipv4(nic)
        if v4:
            ip_s, nm_s = v4
            try:
                net = ipaddress.IPv4Network((ip_s, nm_s), strict=False)
                if net.num_addresses >= 4:
                    if HIDE_DOCKER and any(net.overlaps(n) for n in _docker_nets):
                        continue
                    nets4.append(net)
            except: pass
        v6 = get_iface_ipv6(nic)
        if v6:
            try:
                net = ipaddress.IPv6Network(v6 + "/64", strict=False)
                if HIDE_DOCKER and any(net.overlaps(n) for n in _docker_nets):
                    continue
                nets6.append(net)
            except: pass
    for s in SUBNETS_EXTRA_ENV.split(","):
        s = s.strip()
        if not s: continue
        try:
            if ":" in s:
                net = ipaddress.IPv6Network(s, strict=False)
                nets6.append(net)
            else:
                net = ipaddress.IPv4Network(s, strict=False)
                nets4.append(net)
        except: pass
    # dedup
    def dedup(nets):
        seen = set(); uniq = []
        for n in nets:
            key = str(n)
            if key not in seen:
                seen.add(key); uniq.append(n)
        return uniq
    return dedup(nets4), dedup(nets6)

def try_rdns(ip: str) -> Optional[str]:
    try:
        name, _, _ = socket.gethostbyaddr(ip)
        return name
    except: return None

# ───────────────────────── Estado de dispositivos ─────────────────────────
def _choose_key(mac: Optional[str], ip: Optional[str]) -> Optional[str]:
    if mac and mac_is_valid(mac):
        return mac.lower()
    if ip:
        return f"ip:{ip}"
    return None

def upsert_device(mac: Optional[str], ip: Optional[str] = None, hostname: Optional[str] = None) -> None:
    key = _choose_key(mac, ip)
    if not key:
        return
    ts = now()
    with lock:
        d = devices.get(key, {"mac": key, "first_seen_ever": ts})
        if ip:
            d["ip"] = ip
            d["scope"] = classify_scope(ip)
            ip_to_mac[ip] = key
        if hostname:
            d["hostname"] = hostname
            d.setdefault("name", hostname)
        if mac and mac_is_valid(mac) and not d.get("vendor"):
            d["vendor"] = vendor_from_mac(mac)
        d["online"] = True
        d["last_seen"] = ts
        devices[key] = d

def set_device_ports(ip: str, open_ports: List[int]) -> None:
    key = ip_to_mac.get(ip, f"ip:{ip}")
    with lock:
        d = devices.setdefault(key, {"mac": key, "first_seen_ever": now(), "ip": ip, "online": True})
        d["ip"] = ip
        d["scope"] = classify_scope(ip)
        d["ports"] = sorted(open_ports)
        services = []
        sset = set(open_ports)
        if 1883 in sset: services.append("MQTT")
        if 8883 in sset: services.append("MQTTS")
        if 6379 in sset: services.append("Redis")
        if 9100 in sset: services.append("NodeExporter?")
        if 3000 in sset: services.append("Grafana?")
        if 502 in sset:  services.append("Modbus/TCP")
        if 9200 in sset: services.append("Elasticsearch")
        if 5601 in sset: services.append("Kibana")
        if 80 in sset or 8080 in sset or 443 in sset: services.append("HTTP(S)")
        if 22 in sset: services.append("SSH")
        d["services_guess"] = services
        if RESOLVE_RDNS and ip and not d.get("rdns_checked"):
            rdns = try_rdns(ip)
            if rdns:
                d["rdns"] = rdns
                if not d.get("hostname"):
                    d["hostname"] = rdns
                    d.setdefault("name", rdns)
            d["rdns_checked"] = True

# ───────────────────────── Parsers pasivos ─────────────────────────
def mdns_try_extract_name(pkt) -> Optional[str]:
    try:
        raw = bytes(pkt[Raw]) if Raw in pkt else b""
        i = raw.find(b".local")
        if i > 0:
            frag = raw[max(0, i - 64):i].decode("utf-8", "ignore")
            name = frag.split("\x00")[-1].split()[-1].strip(".")
            return name or None
    except Exception:
        pass
    return None

def handle_packet(pkt) -> None:
    try:
        if ARP in pkt:
            p = pkt[ARP]
            upsert_device(p.hwsrc, ip=p.psrc)
        if UDP in pkt and pkt[UDP].dport == 5353:
            mac = pkt[Ether].src if Ether in pkt else None
            ip = pkt[IP].src if IP in pkt else None
            name = mdns_try_extract_name(pkt)
            upsert_device(mac, ip=ip, hostname=name)
        # ICMPv6 Neighbor Discovery
        if ICMPv6ND_NS in pkt or ICMPv6ND_NA in pkt:
            mac = pkt[Ether].src if Ether in pkt else None
            ip6 = pkt[IPv6].src if IPv6 in pkt else None
            upsert_device(mac, ip=ip6)
    except Exception as e:
        print("[agent] handle_packet err:", repr(e))

# ───────────────────────── Redis flush ─────────────────────────
def make_redis() -> redis.Redis:
    return redis.Redis(
        host=REDIS_HOST,
        port=REDIS_PORT,
        password=(REDIS_PASS or None),
        db=REDIS_DB,
        socket_timeout=2.0,
        socket_connect_timeout=2.0,
        decode_responses=True,
    )

def flush_loop() -> None:
    r = make_redis()
    last_log = 0
    public_ip_cache = None
    last_public_ip_check = 0
    PUBLIC_IP_CHECK_INTERVAL = 300

    while True:
        time.sleep(2.0)
        try:
            current_time = now()
            if current_time - last_public_ip_check >= PUBLIC_IP_CHECK_INTERVAL:
                new_public_ip = get_public_ip()
                if new_public_ip:
                    public_ip_cache = new_public_ip
                    print(f"[agent] IP pública actualizada: {public_ip_cache}")
                last_public_ip_check = current_time

            with lock:
                lst = []
                for d in devices.values():
                    if HIDE_DOCKER and d.get("scope") == "DOCKER":
                        continue
                    lst.append({
                        "ip": d.get("ip"),
                        "mac": d.get("mac"),
                        "name": d.get("name") or d.get("hostname"),
                        "node": "",
                        "topic": "",
                        "vendor": d.get("vendor"),
                        "hostname": d.get("hostname"),
                        "rdns": d.get("rdns"),
                        "uros_agent_host": "",
                        "uros_agent_port": None,
                        "online": True,
                        "first_seen_ever": d.get("first_seen_ever"),
                        "last_seen": d.get("last_seen"),
                        "ports": d.get("ports"),
                        "services_guess": d.get("services_guess"),
                        "sensor": SENSOR_NAME,
                        "scope": d.get("scope"),
                    })
                snap = {
                    "devices": lst,
                    "sensor": SENSOR_NAME,
                    "ts": now(),
                    "public_ip": public_ip_cache
                }
            r.set(REDIS_KEY_DEVICES, json.dumps(snap, ensure_ascii=False))

            t = now()
            if t - last_log >= 5:
                ip_info = f" public_ip={public_ip_cache}" if public_ip_cache else ""
                print(f"[agent] devices={len(lst)} flushed → redis://{REDIS_HOST}:{REDIS_PORT} key={REDIS_KEY_DEVICES} iface={IFACE}{ip_info}")
                last_log = t
        except Exception as e:
            print("[agent] redis err:", repr(e))

# ───────────────────────── Escaneo ARP activo + ICMPv6 ─────────────────────────
def arp_sweep_once() -> Dict[str, List[str]]:
    results: Dict[str, List[str]] = {}
    nets4, nets6 = networks_from_ifaces()
    print(f"[scan] IPv4 nets: {', '.join(map(str, nets4)) or '(none)'}")
    print(f"[scan] IPv6 nets: {', '.join(map(str, nets6)) or '(none)'}")
    # IPv4
    for net in nets4:
        try:
            if net.num_addresses > ARP_MAX_HOSTS:
                print(f"[scan] skip net {net} ({net.num_addresses} hosts) > ARP_MAX_HOSTS={ARP_MAX_HOSTS}")
                continue
            targets = [str(ip) for ip in net.hosts()]
            if not targets:
                continue
            ans, _ = srp(
                ScapyEther(dst="ff:ff:ff:ff:ff:ff")/ScapyARP(pdst=targets),
                timeout=2, retry=1, verbose=0
            )
            found_ips: List[str] = []
            for _snd, rcv in ans:
                mac = (rcv[ScapyEther].src or "").lower()
                ip = rcv[ScapyARP].psrc
                upsert_device(mac, ip=ip)
                found_ips.append(ip)
            results[str(net)] = found_ips
        except Exception as e:
            print("[scan] ARP error on", net, ":", repr(e))
    # IPv6
    for net6 in nets6:
        try:
            # Tomamos solo un /64 para evitar explosión
            hosts = list(net6.hosts())[:256]
            found_ips = []
            for tgt in hosts:
                ans, _ = srp(
                    ScapyEther(dst="ff:ff:ff:ff:ff:ff")/
                    ScapyIPv6(dst=str(tgt))/ScapyICMPv6ND_NS(tgt=str(tgt)),
                    timeout=1, retry=0, verbose=0
                )
                for _snd, rcv in ans:
                    mac = (rcv[ScapyEther].src or "").lower()
                    ip6 = rcv[ScapyIPv6].src
                    upsert_device(mac, ip=ip6)
                    found_ips.append(ip6)
            results[str(net6)] = found_ips
        except Exception as e:
            print("[scan] ICMPv6 ND error on", net6, ":", repr(e))
    return results

def arp_sweep_loop() -> None:
    r = make_redis()
    while True:
        res = arp_sweep_once()
        try:
            r.set("trix:net:scan:last", json.dumps({"ts": now(), "arp": res, "sensor": SENSOR_NAME}))
        except Exception as e:
            print("[scan] redis err:", repr(e))
        time.sleep(SCAN_INTERVAL_ARP)

# ───────────────────────── Mini-portscan TCP (IPv4 + IPv6) ─────────────────────────
def tcp_connect(ip: str, port: int, timeout: float) -> bool:
    try:
        family = socket.AF_INET6 if ":" in ip else socket.AF_INET
        with socket.socket(family, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            s.connect((ip, port))
            return True
    except:
        return False

def collect_ips_for_portscan() -> List[str]:
    with lock:
        ips = []
        for d in devices.values():
            ip = d.get("ip")
            if not ip:
                continue
            if HIDE_DOCKER and d.get("scope") == "DOCKER":
                continue
            ips.append(ip)
    return sorted(set(ips))

def portscan_loop() -> None:
    r = make_redis()
    while True:
        ips = collect_ips_for_portscan()
        if ips:
            print(f"[scan] PORTSCAN on {len(ips)} host(s), ports={PORTS_TCP}")
        for ip in ips:
            open_ports: List[int] = []
            for p in PORTS_TCP:
                if tcp_connect(ip, p, TCP_TIMEOUT):
                    open_ports.append(p)
            set_device_ports(ip, open_ports)
            try:
                r.set(f"trix:net:ports:{ip}", json.dumps({"ts": now(), "ip": ip, "open": open_ports}))
            except Exception as e:
                print("[scan] redis err (ports):", ip, repr(e))
        time.sleep(SCAN_INTERVAL_PORTS)

# ───────────────────────── Sondeo SSDP (multicast) ─────────────────────────
def ssdp_probe_once():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        s.settimeout(2.0)
        msg = "\r\n".join([
            "M-SEARCH * HTTP/1.1",
            "HOST: 239.255.255.250:1900",
            'MAN: "ssdp:discover"',
            "MX: 1",
            "ST: ssdp:all",
            "", ""
        ]).encode("ascii")
        s.sendto(msg, ("239.255.255.250", 1900))
        t_end = time.time() + 2.0
        while time.time() < t_end:
            try:
                data, (rip, _rport) = s.recvfrom(4096)
                txt = data.decode("utf-8", "ignore")
                name = None
                for line in txt.splitlines():
                    L = line.strip()
                    if L.upper().startswith("SERVER:") or L.upper().startswith("USN:"):
                        name = L.split(":",1)[1].strip()
                        break
                key = ip_to_mac.get(rip, f"ip:{rip}")
                with lock:
                    d = devices.setdefault(key, {"mac": key, "first_seen_ever": now(), "ip": rip, "online": True})
                    d["ip"] = rip
                    d["scope"] = classify_scope(rip)
                    if name and not d.get("hostname"):
                        d["hostname"] = name
                        d.setdefault("name", name)
            except socket.timeout:
                break
            except Exception as e:
                print("[mcast] ssdp recv err:", repr(e))
        s.close()
    except Exception as e:
        print("[mcast] ssdp err:", repr(e))

def mcast_probe_loop():
    while True:
        ssdp_probe_once()
        time.sleep(MCAST_PROBE_INTERVAL)

# ───────────────────────── Obtener IP pública (IPv4 o IPv6) ─────────────────────────
def get_public_ip() -> Optional[str]:
    services = [
        "https://api.ipify.org?format=json",
        "https://ifconfig.me/ip",
        "https://icanhazip.com",
        "https://checkip.amazonaws.com"
    ]
    for service in services:
        try:
            response = requests.get(service, timeout=3)
            if response.status_code == 200:
                if "json" in service:
                    return response.json().get("ip", "").strip()
                else:
                    return response.text.strip()
        except Exception as e:
            print(f"[agent] Error obteniendo IP pública de {service}: {repr(e)}")
            continue
    print("[agent] No se pudo obtener la IP pública")
    return None

# ───────────────────────── Main / Sniffer ─────────────────────────
def main() -> None:
    try:
        print("[agent] ifaces:", ", ".join(i for i in get_if_list()))
    except Exception:
        pass

    print(f"[agent] SENSOR_NAME={SENSOR_NAME}")
    print(f"[agent] REDIS={REDIS_HOST}:{REDIS_PORT}/{REDIS_DB} key={REDIS_KEY_DEVICES}")
    print(f"[agent] IFACE={IFACE}")
    initial_public_ip = get_public_ip()
    if initial_public_ip:
        print(f"[agent] IP pública detectada: {initial_public_ip}")
    else:
        print("[agent] No se pudo obtener IP pública al inicio")

    print("[agent] sniffing filters: ARP + mDNS (udp/5353) + ICMPv6 ND")
    print(f"[agent] deep-scan: ARP/ICMPv6 every {SCAN_INTERVAL_ARP}s, PORTS every {SCAN_INTERVAL_PORTS}s, multicast every {MCAST_PROBE_INTERVAL}s")

    threading.Thread(target=flush_loop, daemon=True).start()
    threading.Thread(target=arp_sweep_loop, daemon=True).start()
    threading.Thread(target=portscan_loop, daemon=True).start()
    threading.Thread(target=mcast_probe_loop, daemon=True).start()

    sniff(
        iface=IFACE,
        filter="arp or (udp and port 5353) or (icmp6 and ip6[40] == 135 or ip6[40] == 136)",
        prn=handle_packet,
        store=False,
        promisc=True,
    )

if __name__ == "__main__":
    main()