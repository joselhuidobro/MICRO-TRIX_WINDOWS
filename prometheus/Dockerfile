# Imagen oficial de Prometheus
FROM prom/prometheus:v2.53.0

# Copiamos la configuración y (opcionalmente) reglas
COPY prometheus.yml /etc/prometheus/prometheus.yml
# COPY rules/ /etc/prometheus/rules/

# Ajustes útiles:
#  - Retención 15 días (ajusta según disco)
#  - web.enable-lifecycle permite recargar config vía POST /-/reload
CMD [ \
  "--config.file=/etc/prometheus/prometheus.yml", \
  "--storage.tsdb.path=/prometheus", \
  "--storage.tsdb.retention.time=15d", \
  "--web.enable-lifecycle" \
]

