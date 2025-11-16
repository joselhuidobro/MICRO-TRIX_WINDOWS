# postgres/Dockerfile
FROM postgres:15-alpine

# Instalar extensiones útiles
RUN apk add --no-cache postgis

# Copiar script de inicialización
COPY init.sql /docker-entrypoint-initdb.d/

# Configurar permisos y ownership correcto
RUN chown postgres:postgres /docker-entrypoint-initdb.d/init.sql && \
    chmod 644 /docker-entrypoint-initdb.d/init.sql

# Healthcheck para monitorear el contenedor
HEALTHCHECK --interval=10s --timeout=3s --start-period=30s --retries=3 \
  CMD pg_isready -U ${POSTGRES_USER:-postgres} -h localhost || exit 1

# Configuración optimizada de PostgreSQL
CMD ["postgres", "-c", "log_statement=all", "-c", "log_destination=stderr", "-c", "max_connections=200"]