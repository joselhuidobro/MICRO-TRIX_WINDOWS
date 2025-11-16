FROM debian:bookworm-slim

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      wireguard-go wireguard-tools iptables iproute2 \
      ca-certificates bash \
 && rm -rf /var/lib/apt/lists/* \
 && update-ca-certificates

VOLUME ["/config"]
ENV WG_CONF_PATH=/config/wg0.conf
ENV WG_QUICK_USERSPACE=true
ENV WG_TUN_SUBNET=10.6.0.0/24

COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

HEALTHCHECK --interval=30s --timeout=5s --retries=5 CMD wg show wg0 >/dev/null 2>&1 || exit 1
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

