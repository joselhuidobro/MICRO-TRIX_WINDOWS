# FASE 1: Compilar Hugo dentro de Docker
FROM klakegg/hugo:0.111.3 AS builder
WORKDIR /src
COPY . .
RUN hugo --minify --verbose --debug

# FASE 2: Servir con Nginx
FROM nginx:alpine
COPY --from=builder /src/public /usr/share/nginx/html

COPY nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 8030
CMD ["nginx", "-g", "daemon off;"]