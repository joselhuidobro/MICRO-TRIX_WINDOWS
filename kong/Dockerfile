# Dockerfile
FROM kong:3.9

# Copiamos la config declarativa
COPY kong.yml /etc/kong/kong.yml

# Modo DB-less + puertos que usaremos
ENV KONG_DATABASE=off \
    KONG_DECLARATIVE_CONFIG=/etc/kong/kong.yml \
    KONG_PROXY_LISTEN=0.0.0.0:8000 \
    KONG_STATUS_LISTEN=0.0.0.0:8100 \
    KONG_STREAM_LISTEN="0.0.0.0:30888 udp" \
    KONG_ADMIN_LISTEN=off \
    KONG_PROXY_ACCESS_LOG=/dev/stdout \
    KONG_PROXY_ERROR_LOG=/dev/stderr \
    KONG_PROXY_STREAM_ACCESS_LOG="/dev/stdout basic" \
    KONG_PROXY_STREAM_ERROR_LOG=/dev/stderr

# Exponemos HTTP y status, y el puerto UDP
EXPOSE 8000 8100
EXPOSE 30888/udp

ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["kong", "docker-start"]

