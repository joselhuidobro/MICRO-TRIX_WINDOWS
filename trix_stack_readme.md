
/*───────────────────────────────────
 *  ████████╗██████╗ ██╗██╗  ██╗    
 *  ╚══██╔══╝██╔══██╗██║╚██╗██╔╝   
 *     ██║   ██████╔╝██║ ╚███╔╝     
 *     ██║   ██╔══██╗██║ ██╔██╗     
 *     ██║   ██║  ██║██║██╔╝ ██╗    
 *     ╚═╝   ╚═╝  ╚═╝╚═╝╚═╝  ╚═╝   
 *──────────────────────────────────



Arquitectura (resumen express)


✅ “GitLab corriendo en mi red privada, accesible solo por VPN cuando estoy fuera” → eso sí es pro.

Caddy (edge público)
Único servicio expuesto a Internet (80/443). Termina TLS/HTTP3 y hace reverse proxy hacia tu Nginx interno. También puede rutear /api a Kong.
Beneficio: seguridad (solo un puerto de entrada), certificados automáticos y enrutamiento limpio.

Nginx “trixweb” (origin estático)
Sirve tu sitio TRIX ya renderizado (carpeta public/). No expone puertos al host; solo es accesible por Caddy dentro de la red Docker.
Beneficio: desacopla edge de contenido, simple y rápido.

Hugo builder
Contenedor que compila el repo Hugo → genera HTML/CSS/JS en public/ (volumen). Lo ejecutas bajo demanda (o por CI/CD).
Beneficio: despliegue reproducible; no metes toolchain en el servidor web.

Kong (API Gateway, interno)
Recibe tráfico /api desde Caddy y lo distribuye a servicios internos (Flask, Spring Boot, etc.).
Beneficio: plugins (auth, rate-limit, logs), versión única para todas las APIs.

VPN (WireGuard)
Acceso administrativo/privado a toda la malla (p. ej. 10.6.0.0/24). Los peers entran por WG y ven los servicios internos sin exponerlos a Internet.
Beneficio: operación segura y troubleshooting “como si estuvieras en LAN”.

Redes Docker

web-net: Caddy ↔ (opcional) builder; superficie pública mínima.

trix-net: Nginx, Kong y microservicios (UI Flask, Spring, Redis, EMQX, etc.).
Beneficio: segmentación: lo público separado de lo interno.

(Opcional) Knot DNS
Autoritativo para tu dominio o interno para la VPN. Si es público: abre 53/TCP+UDP y delega NS/glue; si es interno: resuelve nombres de tu malla.



# TRIX Stack — Radar UI, ROS 2/micro‑ROS, Modbus, Observabilidad y VPN (WireGuard)

> **Resumen**: Este README describe cómo visualizar **equipos en la LAN (192.168.x.x)** y **servicios en Docker/trix‑net (172.18.x.x)** en una sola UI (“**Radar**”), cómo verificar el flujo **ROS 2 → micro‑ROS → ESP32**, cómo **enviar Modbus solo al HMI** con IP por variables de entorno (sin hardcode), y cómo operar la capa de **observabilidad** (Prometheus/Grafana/Redis/Node‑RED) y la **VPN WireGuard**. Incluye comandos de operación, diagnóstico y limpieza.

> **Nota**: Se antepone `sudo` a los comandos de Docker/administración. Para llamadas HTTP con `curl`, **no es necesario** `sudo` en la mayoría de los casos.

---

## Índice

1. [Arquitectura y objetivos](#arquitectura-y-objetivos)
2. [Accesos rápidos (localhost)](#accesos-rápidos-localhost)
3. [Modelo de red (fusión 172.18.x.x + 192.168.x.x)](#modelo-de-red-fusión-17218xx--192168xx)
4. [UI “Radar”: endpoints y modos](#ui-radar-endpoints-y-modos)
5. [ROS 2 → micro‑ROS → ESP32 (verificación)](#ros-2--micro-ros--esp32-verificación)
6. [Modbus (solo HMI, IP por variables de entorno)](#modbus-solo-hmi-ip-por-variables-de-entorno)
7. [Observabilidad (Prometheus/Grafana/Redis/Node‑RED)](#observabilidad-prometheusgrafanaredisnode-red)
8. [Redis: claves y salud](#redis-claves-y-salud)
9. [WireGuard: estado, peers y configuración de cliente](#wireguard-estado-peers-y-configuración-de-cliente)
10. [ESP32 con ESP‑IDF: build/flash/monitor](#esp32-con-esp-idf-buildflashmonitor)
11. [Limpieza de Docker (⚠️ con cuidado)](#limpieza-de-docker--con-cuidado)
12. [Operación y logs frecuentes](#operación-y-logs-frecuentes)
13. [Seguridad y buenas prácticas](#seguridad-y-buenas-prácticas)
14. [Snippets útiles (cURL y Prometheus)](#snippets-útiles-curl-y-prometheus)

---






## Arquitectura y objetivos

```text
Cliente (LAN/VPN) ──▶  Kong (30080) ──▶  UI/Flask, Grafana, Prometheus, Node‑RED
         │                 │
         │                 ├──▶  UI “Radar” (funde 192.168.x.x + 172.18.x.x con etiquetas)
         │                 └──▶  Endpoints de telemetría, ROS y salud

ROS 2 (Flask) ──▶ micro‑ROS Agent (UDP) ──▶ ESP32 (micro‑ROS)





# Vector 1: Datos de Robot (micro-ROS)
robot_encoder → /dev/ttyUSB0 → UDP → micro-ROS Agent

# Vector 2: Datos de Red Industrial  
net_sensor_cdmx → ARP/UDP → Redis → REDIS_KEY_DEVICES=trix:udp:devices:last

# Vector 3: Estado Sistema (infraestructura)
Redis (estado) + Host network (acceso total) → Posible acción


Trazabilidad completa: OpenTelemetry (OTel) → Prometheus/Grafana + Jaeger/Tempo.

Memoria de agentes y persistencia: LangGraph persistence + vector DB (pgvector/Qdrant).

Ejecución de modelos desacoplada: vLLM/TGI como “model server” detrás de tu agente.

Evaluación continua: LangSmith/Ragas + tests de regresión de prompts/policies.

Cola/event bus productivo: Redis Streams está bien; NATS/Kafka si escalas.

Secretos & supply-chain: Vault/SOPS + SBOM (Syft/Grype) en tus imágenes.

Despliegue: pasar de Compose a k3s/Kubernetes cuando tengas 3+ nodos.

RBAC y políticas de datos más finas: OPA + Rego con tests y CI.

API Gateway con rate-limits/circuit-breakers (Kong ya lo tienes: configúralo para AI).


Modbus: Flask/HMI (IP por env vars)  ✖  (no enviar al ESP32)
```

**Objetivos clave**
- Visualizar **LAN** y **trix‑net** en una sola página con etiqueta de **origen**.
- Asegurar que la **UI/Flask publica en ROS 2** y llega al **micro‑ROS Agent** (UDP ok).
- **Modbus** limitado al **HMI**, con IP configurable por **variables de entorno**.
- Exponer endpoints de **observabilidad** y comandos de **operación/diagnóstico**.

---


✅ Redes separadas: web-net para servicios públicos, trix-net para internos
✅ Volúmenes persistentes: Definidos centralmente en 00-foundation
✅ Token seguro: Almacenado en .env, no en el código
✅ Healthcheck: El tunnel espera a que Caddy esté listo
✅ Sin puertos expuestos públicamente: Todo va por Cloudflare Tunnel


cloudflare_tunnel:
  command: tunnel --url http://caddy:80  # ← SOLO este endpoint
```

✅ **Accesible**: Tu sitio Hugo servido por Caddy en el puerto 80
- `https://random-xyz.trycloudflare.com/` → Hugo
- `https://random-xyz.trycloudflare.com/docs` → Sphinx docs
- `https://random-xyz.trycloudflare.com/api` → Doxygen API

## Lo que NO es accesible:

❌ Kong (puerto 30080, 8100)
❌ Knot DNS (puerto 5353)
❌ Cualquier otro servicio en `trix-net`
❌ Tu equipo local
❌ Otros contenedores Docker
❌ Bases de datos (PostgreSQL, Redis)
❌ EMQX, Node-RED, Grafana, etc.

## ¿Por qué es seguro?
```
Internet
    ↓
Cloudflare Tunnel (tunnel container)
    ↓
[web-net] ← Solo esta red Docker
    ↓
Caddy:80 ← SOLO este servicio
    ↓
Volúmenes de archivos estáticos (hugo_public, sphinx_html, doxygen_html)
```

**Aislamiento por capas:**

1. **Red `trix-net`**: Completamente aislada, el tunnel ni siquiera está conectado
2. **Red `web-net`**: El tunnel solo puede hablar con Caddy
3. **Caddy**: Solo sirve archivos estáticos desde volúmenes read-only (`:ro`)

## Ejemplo visual de tu arquitectura:
```
┌─────────────────────────────────────────────┐
│         Internet (público)                  │
└─────────────────┬───────────────────────────┘
                  │
          ┌───────▼────────┐
          │ Cloudflare     │
          │ Tunnel         │
          └───────┬────────┘
                  │
    ┌─────────────▼─────────────┐
    │      web-net (aislada)    │
    │                           │
    │    ┌─────────────┐        │
    │    │   Caddy:80  │        │
    │    └──────┬──────┘        │
    │           │               │
    │    Archivos estáticos     │
    │    (solo lectura)         │
    └───────────────────────────┘

    ┌───────────────────────────┐
    │   trix-net (privada)      │
    │                           │
    │  ┌──────┐  ┌──────┐      │
    │  │ Kong │  │ Knot │      │
    │  └──────┘  └──────┘      │
    │                           │
    │  ┌──────┐  ┌──────┐      │
    │  │ EMQX │  │  DB  │      │
    │  └──────┘  └──────┘      │
    └───────────────────────────┘
         ↑
         └── SIN acceso desde Internet


## Accesos rápidos (localhost)

> Si entras por **VPN WireGuard**, reemplaza `localhost:30080` por **`10.6.0.1:30080`**.

- **UI**: `http://localhost:30080/ui`
- **Index (fallback)**: `http://localhost:30080/`
- **Telemetría (UI)**: `http://localhost:30080/telemetry` · `http://localhost:30080/ui/telemetry` · `http://localhost:30080/setpoint`
- **UDP devices**: `http://localhost:30080/udp/devices` (SSE: `/udp/devices/stream`)
- **ROS (tópicos UI)**: `http://localhost:30080/ui/ros/topics`
- **Node‑RED**: `http://localhost:30080/nodered`
- **Grafana**: `http://localhost:30080/grafana`
- **Prometheus (targets/metrics)**: `http://localhost:9090/targets?search=` · `http://localhost:9090/metrics`
- **Redis exporter**: `http://localhost:9121/metrics`
- **Git observer**: `http://localhost:7011/metrics`
- **Otro exporter** (ajusta): `http://localhost:8100/metrics`

---

## Modelo de red (fusión 172.18.x.x + 192.168.x.x)

- **LAN**: `192.168.x.x` (host y dispositivos físicos, p. ej. ESP32)
- **Bridge trix‑net (Docker)**: `172.18.x.x` (servicios en contenedores)
- La UI “Radar” realiza un **merge** etiquetando `source: lan | radar | trix`.

---

## UI “Radar”: endpoints y modos

**Mezcla (LAN + 172.18)**
- JSON principal: `http://localhost:5000/udp/devices` y `http://localhost:30080/udp/devices`
- Snapshot con breakdown: `http://localhost:5000/udp/devices/snapshot`

**Modos de depuración**
- Solo **LAN**: `http://localhost:5000/udp/devices/debug?mode=lan`
- Solo **bridge 172.18** (radar): `http://localhost:5000/udp/devices/debug?mode=radar`
- Solo **contenedores** (Docker SDK): `http://localhost:5000/udp/devices/debug?mode=trix`

**Streams SSE**
- Dispositivos: `http://localhost:30080/udp/devices/stream`
- Orientación: `http://localhost:30080/orientation`

> **Tip**: Si un host no aparece, prueba conectividad (ping/ARP), revisa permisos del escáner y que el servicio publicador esté corriendo.

---

## ROS 2 → micro‑ROS → ESP32 (verificación)

```bash
sudo docker exec -it flask_ui_encoder_robot bash -lc 'source /opt/ros/$ROS_DISTRO/setup.bash && ros2 topic list'
sudo docker exec -it flask_ui_encoder_robot bash -lc 'source /opt/ros/$ROS_DISTRO/setup.bash && ros2 topic info -v /servo_setpoint_deg'
sudo docker exec -it flask_ui_encoder_robot bash -lc 'source /opt/ros/$ROS_DISTRO/setup.bash && ros2 topic info -v /RELAY_telemetry_11B384'
sudo docker exec -it flask_ui_encoder_robot bash -lc 'source /opt/ros/$ROS_DISTRO/setup.bash && ros2 topic info -v /relevadores'
sudo docker exec -it flask_ui_encoder_robot bash -lc 'source /opt/ros/$ROS_DISTRO/setup.bash && ros2 topic info -v /rosout'

# Gráfico de nodos
sudo docker exec -it flask_ui_encoder_robot bash -lc 'source /opt/ros/$ROS_DISTRO/setup.bash; rqt_graph'
```

> **Nota**: Si `ros2` no se encuentra, confirma `which ros2` y que se haya `source`‑ado el `setup.bash`.

---

## Modbus (solo HMI, IP por variables de entorno)

- Enviar **Modbus únicamente al HMI**. Evitar enviar Modbus al ESP32.
- IP/puerto del HMI por **variables de entorno** (ej.: `MODBUS_HMI_IP`, `MODBUS_HMI_PORT`).
- Endpoints UI relacionados: `http://localhost:30080/data` (datos) y `http://localhost:30080/setpoint`.

> **Motivo**: Separación de responsabilidades, menor superficie de ataque y flexibilidad de despliegue.

---

## Observabilidad (Prometheus/Grafana/Redis/Node‑RED)

**Checks rápidos**
```bash
# Node‑RED vía Kong
curl -I http://localhost:30080/nodered

# ¿Grafana up en Prometheus?
curl -G 'http://localhost:9090/api/v1/query' --data-urlencode 'query=up{job="grafana"}'
curl -G 'http://localhost:9090/api/v1/query' --data-urlencode 'query=sum by (job) (up)'
```

**Dashboards/Targets**
- Grafana: `http://localhost:30080/grafana`
- Prometheus targets: `http://localhost:9090/targets?search=`

> **Tip**: Añade `wireguard`, `ui`, `nodered`, `redis_exporter`, etc. como **jobs** para tener una vista por servicio (`sum by (job) (up)`).

---

## Redis: claves y salud

```bash
sudo docker exec -it redis_trix redis-cli -a trixredis123 GET trix:udp:devices:last
sudo docker exec -it redis_trix redis-cli -a trixredis123 INFO persistence
sudo docker exec -it redis_trix redis-cli -a trixredis123 INFO stats
```

- Salud en UI: `http://localhost:30080/redis/health`
- La clave `trix:udp:devices:last` debe devolver un JSON/DICT con `devices > 0`.

---

## WireGuard: estado, peers y configuración de cliente

**Inspección y logs**
```bash
sudo docker ps --format "{{.Names}}  {{.Ports}}" | grep wireguard
sudo docker inspect wireguard | grep -A4 '"Ports"'
sudo docker logs wireguard | tail -n 80
```

**Estado de interfaz/handshakes**
```bash
sudo docker exec -it wireguard bash -lc 'wg show; wg show wg0 latest-handshakes'
sudo docker exec -it wireguard bash -lc 'ip addr show wg0; wg show'
sudo docker exec -it wireguard bash -lc 'ls -R /config'
sudo docker exec -it wireguard bash -lc 'ping -c 3 10.6.0.2'
```
VPN
http://10.6.0.1:30080/ui
http://10.6.0.1:9090/targets?search=`
http://10.6.0.1/:30080/redis/health
http://10.6.0.1:30080/nodered
http://10.6.0.1:8080/
http://10.6.0.1:30080/grafana

http://10.6.0.1:6800/login/


http://10.6.0.1:3000/grafana/metrics
http://10.6.0.1:9090/metrics


docker exec -it knot knotc conf-check
docker exec -it knot kzonecheck /zones/example.test.zone

 sudo docker exec -it kong_db_less getent hosts trix-erp

docker logs -f postgres_trix | sed -n '1,120p'

sudo docker logs cloudflare_tunnel | grep -Eo 'https://[a-z0-9.-]*trycloudflare.com' | head -1


# y prueba siguiendo el redirect:



sudo ansible-playbook -i hosts.ini site.yml --syntax-check

wg genkey | tee server_private.key | wg pubkey > server_public.key
# Crear/reescribir vault.yml con la clave privada WG
sudo bash -c 'printf -- "---\nwg_private_key: \"%s\"\n" "$(tr -d "\n" < server_private.key)" > vault.yml'

# Verifica el contenido (debe verse YAML con una sola línea de clave)
cat vault.yml


hostname -I
ip -4 addr | grep -A2 -E 'state UP|inet '

sudo wg show
curl -4 ifconfig.me      # en el CEL con la VPN activa debe ver la IP pública del servidor
# en el servidor:
sudo iptables -t nat -S | grep MASQUERADE
sudo iptables -S FORWARD | grep -E 'wg0|RELATED,ESTABLISHED'
sudo tcpdump -ni eth0 udp port 51820 


sudo # Ver qué config está usando
docker exec -it kiali sh -c 'echo $CONFIG_FILE && sed -n "1,60p" $CONFIG_FILE'

# Ver env efectivas
docker inspect kiali --format '{{range .Config.Env}}{{println .}}{{end}}' | egrep 'CONFIG_FILE|AUTH_STRATEGY|LOGIN_TOKEN'

cd ~/Documents/micro-trix_2/deploy
sudo ansible-playbook -i hosts.ini site.yml -l jetson --ask-vault-pass



 sudo systemctl status wg-quick@wg0 --no-pager


 curl -4 ifconfig.me ; echo
201.141.100.234
ip -4 addr show eth0 | grep inet
    inet 192.168.0.82/24 brd 192.168.0.255 scope global dynamic noprefixroute eth0



**Ver mounts/env de wireguard**
```bash
sudo docker inspect -f '{{range .Config.Env}}{{println .}}{{end}}' wireguard | grep WG_QUICK_USERSPACE || echo "NO_ESTA"
sudo docker inspect wireguard --format '{{range .Mounts}}{{println .Type " " .Source " -> " .Destination}}{{end}}'
```

**Sanitizar y mostrar `wg0.conf`**
```bash
sed -n '1,200p' ./wireguard/wireguard_config/wg0.conf \
 | sed 's/PrivateKey.*/PrivateKey=<hidden>/' \
 | sed 's/PublicKey.*/PublicKey=<hidden>/'
```

**Generar QR local (ej. `peer1.conf`)**
```bash
qrencode -t ansiutf8 < peer1.conf
```

**Plantilla: configuración de celular (WireGuard App)**

**Interface**
- **Name**: `ui2` (o el que prefieras)
- **Private key**: generada por la app
- **Address**: `10.6.0.2/32`
- **DNS**: `1.1.1.1`
- **MTU**: `1420`

**Peer**
- **Public key**: *del servidor (en `wg0.conf`)*
- **AllowedIPs**: `10.6.0.0/24,192.168.0.0/24`
- **Endpoint**: `X.X.X.X:51820` *(IP/host público real + puerto)

**Ruta vía VPN**
```
Cliente (cel/PC) → 10.6.0.1:30080 (host wg0) → Kong → ui:5000, grafana:3000, prometheus:9090, ...
```

> **Tip**: Usa `wg show wg0 latest-handshakes` para ver si hubo intercambio reciente (epoch seconds por peer).

---

## ESP32 con ESP‑IDF: build/flash/monitor

```bash
# Build limpio
sudo docker exec -it ros_relevadores_y_encoder bash -lc '
  if [ -f /opt/esp/idf/export.sh ]; then source /opt/esp/idf/export.sh;
  elif [ -f /opt/esp-idf/export.sh ]; then source /opt/esp-idf/export.sh; fi;
  cd /esp2024 && idf.py fullclean && idf.py build'

# Flash
a
sudo docker exec -it ros_relevadores_y_encoder bash -lc '
  if [ -f /opt/esp/idf/export.sh ]; then source /opt/esp/idf/export.sh;
  elif [ -f /opt/esp-idf/export.sh ]; then source /opt/esp-idf/export.sh; fi;
  cd /esp2024 && idf.py -p /dev/ttyUSB0 flash'

# Monitor
sudo docker exec -it ros_relevadores_y_encoder bash -lc '
  if [ -f /opt/esp/idf/export.sh ]; then source /opt/esp/idf/export.sh;
  elif [ -f /opt/esp-idf/export.sh ]; then source /opt/esp-idf/export.sh; fi;
  cd /esp2024 && idf.py -p /dev/ttyUSB0 monitor'

# Comprobar dispositivo y permisos dentro del contenedor
sudo docker exec -it ros_relevadores_y_encoder bash -lc 'ls -l /dev/ttyUSB0; id; groups'
```

> **Nota**: Asegúrate de mapear `/dev/ttyUSB0` al contenedor (y grupos `dialout`/`uucp` según distro).

---

## Limpieza de Docker (⚠️ con cuidado)

```bash
# Parar todo
a
sudo docker stop $(sudo docker ps -aq)

# Borrar contenedores y volúmenes (no en uso)
sudo docker rm -f $(sudo docker ps -aq) 2>/dev/null
sudo docker volume rm $(sudo docker volume ls -q) 2>/dev/null

# Prune (imágenes, cachés y volúmenes no usados)
sudo docker system prune -af
sudo docker system prune -a --volumes
```

> **Advertencia**: Esto elimina **recursos no usados** globalmente. Revisa si hay volúmenes/imágenes que debas conservar.

---

## Operación y logs frecuentes

```bash
# Logs
sudo docker logs git_observer
sudo docker logs -f net_sensor_cdmx
sudo docker logs -n 50 net_sensor_cdmx
sudo docker logs -f micro_ros_agent_HMI2
sudo docker logs flask_ui_encoder_robot
sudo docker logs wireguard | tail -n 80

# Entrar a contenedores
sudo docker exec -it wireguard bash
sudo docker exec -it flask_ui_encoder_robot bash
sudo docker exec -it flask_ui_encoder_robot bash -lc "source /opt/ros/\$ROS_DISTRO/setup.bash"
```

---

## Seguridad y buenas prácticas

- **Modbus**: limitar al HMI; si expones fuera, usa **listas de control**/firewall y segmentación.
- **Redis**: si se accede más allá de `localhost/VPN`, usa **ACL**/contraseñas fuertes y `bind` estricto.
- **WireGuard**: fija puerto (p.ej. `51820/UDP`), monitorea **handshakes** y rota claves cuando proceda.
- **Kong**: protege rutas con **auth**/rate‑limit si expones a internet; usa certificados válidos.

---

## Snippets útiles (cURL y Prometheus)

```bash
# Node‑RED (vía Kong)
curl -I http://localhost:30080/nodered

# Prometheus: estado de Grafana
curl -G 'http://localhost:9090/api/v1/query' --data-urlencode 'query=up{job="grafana"}'

# Prometheus: up por job
curl -G 'http://localhost:9090/api/v1/query' --data-urlencode 'query=sum by (job) (up)'

# Clave Redis desde Flask (debug)
curl http://localhost:5000/debug/redis-key

# Radar: modos
a
curl "http://localhost:5000/udp/devices/debug?mode=lan"
curl "http://localhost:5000/udp/devices/debug?mode=radar"
curl "http://localhost:5000/udp/devices/debug?mode=trix"

# JSON mezclado y snapshot
a
curl "http://localhost:5000/udp/devices"
curl "http://localhost:5000/udp/devices/snapshot"
```

---

### Apéndice: utilidades WireGuard

```bash
# Mostrar wg0 y últimas negociaciones
a
sudo docker exec -it wireguard bash -lc 'wg show; wg show wg0 latest-handshakes'

# Revisar puertos/mounts de wireguard
a
sudo docker ps --format "{{.Names}}  {{.Ports}}" | grep wireguard
sudo docker inspect wireguard | grep -A4 '"Ports"'
sudo docker inspect wireguard --format '{{range .Mounts}}{{println .Type " " .Source " -> " .Destination}}{{end}}'

# Generar QR a partir de peer1.conf (en host)
qrencode -t ansiutf8 < peer1.conf
```

---

**Mantenimiento**: si deseas, podemos añadir una tabla de **estado** (jobs up/down), badges, y un pequeño **diagrama** en Mermaid para la topología.


# Replicación del stack DRON UI + micro-ROS Agent (Minikube)

2025master-DRONespui/
├─ 2025_DOCKER-dronui/
│  ├─ Dockerfile
│  └─ app/...
├─ k8s/
│  ├─ base/
│  │  ├─ kustomization.yaml
│  │  ├─ app_ui/
│  │  │  ├─ deployment.yaml
│  │  │  └─ service.yaml
│  │  └─ micro-ros-agent/
│  │     ├─ deployment.yaml
│  │     └─ service.yaml
│  └─ overlays/
│     └─ minikube/
│        ├─ kustomization.yaml
│        ├─ patch-app-ui-cmd.yaml
│        └─ patch-app-ui-imagepolicy.yaml
├─ scripts/
│  ├─ dev_up.sh
│  ├─ dev_down.sh
│  └─ dev_logs.sh
└─ README_replicacion.md


## Prerrequisitos
- Docker, kubectl, minikube, kustomize (kubectl >=1.14 ya trae `-k`)
- Recursos del proyecto:
  - `2025_DOCKER-dronui/Dockerfile`
  - `2025_DOCKER-dronui/app/...`
  - `k8s/` (esta carpeta)

## Levantar (DEV Minikube)
```bash
scripts/dev_up.sh
# abre la UI:
# http://$(minikube ip):30500/



sudo docker system prune -a
docker system prune -a --volumes
sudo docker-compose build
docker exec -it robot bash

source /opt/esp-idf/export.sh


oermitir a docker abrir gzebo


echo $DISPLAY || true
export DISPLAY=${DISPLAY:-:0}
xhost +SI:localuser:root   # si no jala, usa: xhost +local:root
----------------------------------------


kubectl -n trix get deploy
kubectl -n trix rollout status deploy/ui
kubectl -n trix rollout status deploy/micro-ros-agent
kubectl -n trix rollout status deploy/springboot-api

...

# Entorno mínimo para esta shell
source /opt/ros/$ROS_DISTRO/setup.bash
source /usr/share/gazebo/setup.sh
export ROS_DOMAIN_ID=88
export GAZEBO_MASTER_URI=http://127.0.0.1:11347

# Relanza
/usr/local/bin/gz_up.sh

# Verifica que YA están los servicios
ros2 service list | grep -E 'spawn_entity|delete_entity|set_entity_state|get_entity_state'
------

crear y entrar al pod y container 

kubectl -n trix run ui-debug --image=trix/ui:dev --restart=Never --command -- sh -lc '
  echo "CWD=$(pwd)"; echo; 
  ls -R /app | sed -n "1,200p"; 
  python - <<PY
import os
print("\\nPY files under /app:")
for r,_,f in os.walk("/app"):
  for x in f:
    if x.endswith(".py"):
      print(os.path.join(r,x))
PY
  sleep 3600
'


kubectl -n trix exec -it ui-debug -- sh
# (cuando termines: kubectl -n trix delete pod ui-debug --force --grace-period=0)

---------------------------------------------
kubectl create ns trix 2>/dev/null || true
kubectl config set-context --current --namespace=trix


kubectl apply -f .
kubectl get pods -w
kubectl get deploy
kubectl rollout status deploy/<NOMBRE_DEL_DEPLOY>

----
1) Borra por lotes, sin esperar, y en paralelo
Esto evita que kubectl se “ahogue” intentando borrar todo a la vez.
# A) Workloads primero (sin esperar)
for k in deploy sts ds job cronjob rs pod; do
  kubectl delete "$k" --all --wait=false || true
done

# B) Servicios y red
for k in svc ing networkpolicy; do
  kubectl delete "$k" --all --wait=false --ignore-not-found || true
done

# C) Config/secrets y storage
kubectl delete cm,secret --all --wait=false || true
kubectl delete pvc --all --wait=false --ignore-not-found || true

---------------
kubectl get pods -l app=ui


POD=$(kubectl get pods -l app=ui -o jsonpath='{.items[0].metadata.name}')
kubectl logs $POD --tail=200
kubectl logs $POD --previous --tail=200 || true
--------------------
 ros2 run xacro xacro /sim/models/robotio/robotio.urdf.xacro   mesh_root:=/sim/models/robotio   -o /tmp/robotio.urdf
ros2 run gazebo_ros spawn_entity.py   -entity robotio   -file /tmp/robotio.urdf   -x 0 -y 0 -z 0.2

ros2 service list | grep -E 'spawn_entity|delete_entity|set_entity_state|get_entity_state'



# Apuntar al nuevo tag
kubectl set image deploy/ui ui=joselhuidobro/flask_ui_robot:v4

# Evitar pulls remotos (usa la imagen local de Minikube)
kubectl patch deploy ui -p '{"spec":{"template":{"spec":{"containers":[{"name":"ui","imagePullPolicy":"IfNotPresent"}]}}}}'

# Borra el Pod que estaba en ImagePullBackOff para reintentar
kubectl delete pod -l app=ui

# Ver el rollout
kubectl rollout status deploy/ui



kubectl port-forward svc/ui 8080:5500
# Abre http://localhost:8080


URDF=/tmp/robotio.urdf

# (Re)genera URDF si hace falta
test -f "$URDF" || ros2 run xacro xacro /sim/models/robotio/robotio.urdf.xacro \
  mesh_root:=/sim/models/robotio -o "$URDF"

# Levanta RSP en segundo plano (necesario para que el plugin arranque)
{
  echo "robot_state_publisher:"
  echo "  ros__parameters:"
  echo "    use_sim_time: true"
  echo "    robot_description: |"
  sed 's/^/      /' "$URDF"
} > /tmp/rsp.yaml

( ros2 run robot_state_publisher robot_state_publisher \
    --ros-args --params-file /tmp/rsp.yaml >/tmp/rsp.log 2>&1 ) &

# Respawnea el modelo (por si ya estaba); ignora error si no existe
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'robotio'}" || true
ros2 run gazebo_ros spawn_entity.py -entity robotio -file "$URDF" -x 10 -y 0 -z 0.20

--------------------------------

kubectl exec -it "$POD" -- bash -lc '
> echo "ROS_DISTRO=$ROS_DISTRO"
> source /opt/ros/$ROS_DISTRO/setup.bash || { echo "no se pudo sourcear ROS"; exit 1; }
> python3 - <<PY
> import sys
> print("python:", sys.version)
> try:
>     import rclpy
>     print("rclpy OK")
> except Exception as e:
>     print("rclpy FAIL:", e)
>     import pprint; pprint.pp(sys.path)
>     raise
> PY
> '
ROS_DISTRO=humble
python: 3.10.12 (main, May 27 2025, 17:12:29) [GCC 11.4.0]
rclpy OK
jose@jose-K45VD:~/Descargas/2025_robo



 kubectl exec -it "$POD" -- bash -lc '
> source /opt/ros/$ROS_DISTRO/setup.bash
> export PYTHONPATH="/app:$PYTHONPATH"
> hypercorn app:app -b 0.0.0.0:5500 --workers 1 --log-level debug --access-log -
> '
{"ts": "2025-10-04T17:15:33.748Z", "level": "info", "svc": "trix-ui", "host": "ui-7469d449dc-rth87", "pid": 154, "thread": "Thread-5 (_ws_broadcast_loop)", "subsystem": "WS", "req_id": null, "msg": "[WS] Broadcast loop iniciado", "name": "app", "levelname": "INFO", "levelno": 20, "pathname": "/app/app.py", "filename": "app.py", "module": "app", "exc_info": null, "exc_text": null, "stack_info": null, "lineno": 178, "funcName": "_ws_broadcast_loop", "created": 1759598133.74817, "msecs": 748.0, "relativeCreated": 566.4288997650146, "threadName": "Thread-5 (_ws_broadcast_loop)", "processName": "SpawnProcess-1", "process": 154}
{"ts": "2025-10-04T17:15:33.748Z", "level": "info", "svc": "trix-ui", "host": "ui-7469d449dc-rth87", "pid": 154, "thread": "Thread-6 (_ws_udp_snapshot_loop)", "subsystem": "WS", "req_id": null, "msg": "[WS] UDP snapshot mirror loop iniciado (2.0s)", "name": "app", "levelname": "INFO", "levelno": 20, "pathname": "/app/app.py", "filename": "app.py", "module": "app", "exc_info": null, "exc_text": null, "stack_info": null, "lineno": 190, "funcName": "_ws_udp_snapshot_loop", "created": 1759598133.7486968, "msecs": 748.0, "relativeCreated": 566.9558048248291, "threadName": "Thread-6 (_ws_udp_snapshot_loop)", "processName": "SpawnProcess-1", "process": 154}



kubectl exec -it "$POD" -- bash -lc '
> source /opt/ros/$ROS_DISTRO/setup.bash
> export PYTHONPATH="/app:$PYTHONPATH"
 hypercorn app:app -b 0.0.0.0:5500 --workers 1 --log-level debug --access-log -
 '

---------------------


















