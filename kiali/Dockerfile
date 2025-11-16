# Usa la imagen oficial de Kiali (cámbiala a una versión fija si prefieres)
ARG KIALI_VERSION=latest
FROM quay.io/kiali/kiali:${KIALI_VERSION}

# Kiali corre como no-root en la imagen oficial
# Copiamos nuestro config y apuntamos la var de entorno
COPY config.yaml /opt/kiali/config.yaml
ENV CONFIG_FILE=/opt/kiali/config.yaml  
EXPOSE 20001



