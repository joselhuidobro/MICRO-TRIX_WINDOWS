# ---- Stage 1: prepare site folder ------------------------------------------
FROM alpine:3.20 AS prep
WORKDIR /work

# Copia todo tu repo al contenedor
# (Asegúrate de construir desde la raíz de TRIXWEB)
COPY . /work

# Tu estructura trae todo dentro de content/, así que publicaremos content/ como raíz.
# También cuidamos que exista un index.
RUN set -eux; \
    mkdir -p /out; \
    if [ -d "/work/content" ]; then cp -a /work/content/. /out/; fi; \
    # Si alguien puso assets fuera de content/ por error, intenta sumarlos si existen:
    if [ -d "/work/static" ]; then cp -a /work/static/. /out/; fi; \
    if [ -f /out/index.html ]; then echo "index ok"; else echo "Falta content/index.html" >&2; exit 1; fi

# ---- Stage 2: runtime (NGINX) ----------------------------------------------
FROM nginx:1.27-alpine

# Directorio web
ENV WEB_ROOT=/usr/share/nginx/html
WORKDIR /usr/share/nginx

# Tipos MIME extra (GLB, GLTF, WASM, etc.) y compresión
RUN set -eux; \
  apk add --no-cache bash; \
  rm -f /etc/nginx/conf.d/default.conf; \
  { \
    echo 'types {'; \
    echo '  application/wasm                            wasm;'; \
    echo '  model/gltf-binary                           glb;'; \
    echo '  model/gltf+json                             gltf;'; \
    echo '  image/webp                                  webp;'; \
    echo '  image/avif                                  avif;'; \
    echo '}'; \
  } > /etc/nginx/mime.types

# Config NGINX de sitio (gzip, cache estática, rangos para .glb, SPA fallback opcional)
RUN set -eux; \
  { \
    echo 'server {'; \
    echo '  listen 8030;'; \
    echo '  server_name _;'; \
    echo '  root /usr/share/nginx/html;'; \
    echo '  index index.html;'; \
    echo ''; \
    echo '  # Gzip básico'; \
    echo '  gzip on;'; \
    echo '  gzip_types text/plain text/css application/javascript application/json image/svg+xml;'; \
    echo '  gzip_min_length 1024;'; \
    echo ''; \
    echo '  # Archivos estáticos con cache'; \
    echo '  location ~* \.(?:css|js|png|jpg|jpeg|gif|webp|avif|svg|ico)$ {'; \
    echo '    access_log off;'; \
    echo '    add_header Cache-Control "public, max-age=31536000, immutable";'; \
    echo '    try_files $uri =404;'; \
    echo '  }'; \
    echo ''; \
    echo '  # Modelos 3D: permitir Range para streaming parcial'; \
    echo '  location ~* \.(?:glb|gltf)$ {'; \
    echo '    types { model/gltf-binary glb; model/gltf+json gltf; }'; \
    echo '    add_header Accept-Ranges bytes always;'; \
    echo '    try_files $uri =404;'; \
    echo '  }'; \
    echo ''; \
    echo '  # Páginas HTML y raíz'; \
    echo '  location / {'; \
    echo '    try_files $uri $uri/ /index.html;'; \
    echo '  }'; \
    echo ''; \
    echo '  # Salud/healthcheck'; \
    echo '  location = /health {'; \
    echo '    add_header Content-Type text/plain; return 200 "ok\n";'; \
    echo '  }'; \
    echo '}'; \
  } > /etc/nginx/conf.d/site.conf

# Copia el sitio construido
COPY --from=prep /out/ ${WEB_ROOT}/

# Buenas prácticas: usuario no-root
RUN adduser -D -H -u 10001 web && chown -R web:web ${WEB_ROOT}
USER web

EXPOSE 8030

# Healthcheck liviano
HEALTHCHECK --interval=30s --timeout=3s --start-period=10s --retries=3 \
  CMD wget -qO- http://127.0.0.1:8030/health || exit 1

CMD ["nginx", "-g", "daemon off;"]

