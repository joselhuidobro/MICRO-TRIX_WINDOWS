FROM alpine:3.20 AS downloader
ARG HUGO_VERSION=0.135.0
RUN apk add --no-cache curl tar ca-certificates
RUN arch=$(apk --print-arch) && \
    case "$arch" in aarch64) HUGO_ARCH="ARM64" ;; x86_64) HUGO_ARCH="64" ;; \
      *) echo "Arquitectura no soportada: $arch" && exit 1 ;; esac && \
    curl -fsSL -o /tmp/hugo.tgz \
      "https://github.com/gohugoio/hugo/releases/download/v${HUGO_VERSION}/hugo_extended_${HUGO_VERSION}_Linux-${HUGO_ARCH}.tar.gz" && \
    mkdir -p /out && tar -xzf /tmp/hugo.tgz -C /out hugo

FROM alpine:3.20
# git y go ayudan si tu tema usa Hugo Modules; node/npm cubren PostCSS si hiciera falta
RUN apk add --no-cache ca-certificates libstdc++ nodejs npm git go
COPY --from=downloader /out/hugo /usr/local/bin/hugo
WORKDIR /src
ENTRYPOINT ["sh","-lc"]

