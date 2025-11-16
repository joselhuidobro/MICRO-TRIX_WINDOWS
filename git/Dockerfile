# ./git/Dockerfile
FROM alpine:3.20
RUN apk add --no-cache git git-lfs inotify-tools coreutils busybox-extras
WORKDIR /
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
EXPOSE 7011
HEALTHCHECK --interval=15s --timeout=3s --retries=5 \
  CMD /bin/busybox wget -qO- http://127.0.0.1:7011/metrics || exit 1
ENTRYPOINT ["/entrypoint.sh"]

