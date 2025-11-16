#!/usr/bin/env bash
set -euo pipefail
WG_CONF_PATH="${WG_CONF_PATH:-/config/wg0.conf}"
WG_QUICK_USERSPACE="${WG_QUICK_USERSPACE:-true}"
WG_TUN_SUBNET="${WG_TUN_SUBNET:-10.6.0.0/24}"

echo "[wireguard] starting…"
echo "  WG_CONF_PATH=${WG_CONF_PATH}"
echo "  WG_QUICK_USERSPACE=${WG_QUICK_USERSPACE}"
echo "  WG_TUN_SUBNET=${WG_TUN_SUBNET}"

[ -c /dev/net/tun ] || { echo "ERROR: falta /dev/net/tun (móntalo desde el host)"; exit 1; }
[ -f "${WG_CONF_PATH}" ] || { echo "ERROR: falta ${WG_CONF_PATH} (monta tu wg0.conf en /config)"; exit 1; }

if [[ "${WG_QUICK_USERSPACE,,}" =~ ^(true|1|on)$ ]]; then
  export WG_QUICK_USERSPACE=1
  command -v wireguard-go >/dev/null || { echo "ERROR: falta wireguard-go"; exit 1; }
  echo "[wireguard] userspace habilitado (wireguard-go)"
fi

wg-quick down "${WG_CONF_PATH}" >/dev/null 2>&1 || true
echo "[wireguard] levantando wg0…"
wg-quick up "${WG_CONF_PATH}"

if ! iptables -t nat -C POSTROUTING -s "${WG_TUN_SUBNET}" -o eth0 -j MASQUERADE 2>/dev/null; then
  iptables -t nat -A POSTROUTING -s "${WG_TUN_SUBNET}" -o eth0 -j MASQUERADE
  echo "[wireguard] NAT MASQUERADE agregado para ${WG_TUN_SUBNET} -> eth0"
fi

_term(){ echo "[wireguard] bajando wg0…"; wg-quick down "${WG_CONF_PATH}" || true; exit 0; }
trap _term TERM INT

echo "[wireguard] listo. Estado:"
wg show || true
tail -f /dev/null

