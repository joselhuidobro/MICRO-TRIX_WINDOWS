# syntax=docker/dockerfile:1.6

############################
# Build con Maven preinstalado
############################
FROM maven:3.9-eclipse-temurin-21 AS build
WORKDIR /app

# Asegurar certificados y herramientas mínimas (por si la base es mínima)
USER root
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates curl \
 && update-ca-certificates && rm -rf /var/lib/apt/lists/*

# Opcional: forzar IPv4 desde Java/Maven para evitar issues de IPv6 en algunas redes/NAT
ENV JAVA_TOOL_OPTIONS="-Djava.net.preferIPv4Stack=true"

# Opcional: settings.xml con mirror o timeouts (descomenta si lo usas)
# COPY .ci/maven/settings.xml /root/.m2/settings.xml

# 1) Pre-caché de dependencias con BuildKit (más estable y rápido)
COPY pom.xml .
RUN --mount=type=cache,target=/root/.m2 \
    mvn -B -U -DskipTests \
      -Dmaven.wagon.http.retryHandler.count=5 \
      -Dmaven.wagon.http.pool=false \
      dependency:go-offline

# 2) Copiar código y compilar usando el mismo cache
COPY src ./src
RUN --mount=type=cache,target=/root/.m2 \
    mvn -B -U -DskipTests \
      -Dmaven.wagon.http.retryHandler.count=5 \
      -Dmaven.wagon.http.pool=false \
      package

############################
# Runtime ligero (solo JRE)
############################
FROM eclipse-temurin:21-jre AS runtime
WORKDIR /app

# Copiamos el jar compilado
COPY --from=build /app/target/*.jar /app/app.jar

EXPOSE 8080

# Opcional: healthcheck simple (requiere que tu app exponga /actuator/health o similar)
# HEALTHCHECK --interval=30s --timeout=3s --start-period=20s --retries=3 \
#   CMD curl -fsS http://localhost:8080/actuator/health || exit 1

ENTRYPOINT ["java","-jar","/app/app.jar"]
