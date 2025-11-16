# Dockerfile - Knot DNS (autoritativo, Alpine)
FROM alpine:3.20

# Paquetes (incluye dig para healthcheck)
RUN apk add --no-cache knot knot-utils bind-tools tzdata

# Crea user/group solo si no existen (idempotente)
# Nota: a veces el paquete knot ya trae el user/group.
RUN (getent group knot >/dev/null 2>&1 || addgroup -S knot) \
 && (getent passwd knot >/dev/null 2>&1 || adduser -S -D -H -G knot knot)

# Directorios y permisos
RUN mkdir -p /etc/knot /var/lib/knot /var/log/knot /zones /run/knot \
 && chown -R knot:knot /etc/knot /var/lib/knot /var/log/knot /zones /run/knot

# Copia tu config y zona (ajusta rutas a tu repo real)
COPY knot.conf /etc/knot/knot.conf
COPY zones/ /zones/
RUN chown -R knot:knot /etc/knot /zones

EXPOSE 53/udp 53/tcp 953/tcp

HEALTHCHECK --interval=30s --timeout=5s --retries=5 \
  CMD dig @127.0.0.1 example.test SOA +time=2 +retry=0 || exit 1

USER knot

ENTRYPOINT ["knotd","-c","/etc/knot/knot.conf"]

