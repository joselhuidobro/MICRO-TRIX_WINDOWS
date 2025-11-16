# 🔌 Mapa de Puertos TRIX

**Última actualización:** 2025-01-15  
**Total de puertos:** 29 puertos TCP + 3 UDP  
**Proyecto:** TRIX - Sistema Integrado de Robótica, IoT y Monitoreo

> ⚠️ **IMPORTANTE:** Antes de agregar un nuevo puerto, consulta esta tabla para evitar conflictos.

---

## 📋 Índice Rápido

- [Tabla Completa de Puertos](#-tabla-completa-de-puertos-por-compose)
- [Conflictos Conocidos](#-conflictos-detectados-crítico)
- [Soluciones Aplicadas](#-soluciones-aplicadas)
- [Resumen por Rangos](#-resumen-de-puertos-por-rango)
- [Comandos de Validación](#-comandos-de-validación)
- [Checklist Pre-Deploy](#-checklist-pre-deploy)

---

## 📊 Tabla Completa de Puertos por Compose

### 🏗️ 00-foundation.yml (Infraestructura Base)
| Puerto | Servicio | Protocolo | Descripción | Container | Status |
|--------|----------|-----------|-------------|-----------|--------|
| 6379 | redis | TCP | Cache, pub/sub, sessions | redis_trix | ✅ OK |

**Dependencias:** Ninguna  
**Inicia primero:** Sí  
**Comando:** `trix up-foundation`

---

### 🗄️ 10-data.yml (Bases de Datos)
| Puerto | Servicio | Protocolo | Descripción | Container | Status |
|--------|----------|-----------|-------------|-----------|--------|
| 5432 | postgres | TCP | PostgreSQL principal | postgres_trix | ✅ OK |
| 9070 | pgadmin | TCP | Admin web PostgreSQL | pgadmin_trix | ✅ OK |

**Dependencias:** redis  
**Healthcheck:** PostgreSQL tiene healthcheck crítico  
**Comando:** `trix up-data`

---

### 📡 20-messaging.yml (Message Brokers)
| Puerto | Servicio | Protocolo | Descripción | Container | Status |
|--------|----------|-----------|-------------|-----------|--------|
| 1883 | emqx | TCP | MQTT broker (sin TLS) | emqx_broker | ✅ OK |
| 8083 | emqx | TCP | MQTT over WebSocket | emqx_broker | ✅ OK |
| 18083 | emqx | TCP | EMQX Dashboard Web | emqx_broker | ✅ OK |
| 9999 | micro_ros_agent_hmi | UDP | Micro-ROS Agent UDP | micro_ros_agent_HMI2 | ✅ OK |

**Dependencias:** redis  
**Usuario EMQX:** admin / trix1234  
**Comando:** `trix up-messaging`

---

### 🚀 30-apps.yml (Aplicaciones)
| Puerto | Servicio | Protocolo | Descripción | Container | Status |
|--------|----------|-----------|-------------|-----------|--------|
| 5000 | ui (Flask) | TCP | UI Robot Dashboard | flask_ui_encoder_robot | ✅ OK |
| 5002 | ui (Flask) | UDP | UDP data receiver | flask_ui_encoder_robot | ✅ OK |
| 1880 | node_red | TCP | Node-RED Flow Editor | node_red_trix | ✅ OK |
| 8080 | springboot_api | TCP | REST API Java 21 | springboot_api | ⚠️ CONFLICTO |
| 6800 | trix-erp | TCP | ERP Custom/Odoo | trix-erp | ✅ OK |
| 8030 | trixweb | TCP | Nginx sitio estático | trix-pagina | ✅ OK |
| 7011 | git_observer | TCP | Git utilities | git_observer | ✅ OK |

**Dependencias:** redis, emqx, postgres  
**Nota:** Spring Boot en 8080 conflictúa con cAdvisor  
**Comando:** `trix up-apps`

---

### 📊 40-observability.yml (Monitoreo y Logs)
| Puerto | Servicio | Protocolo | Descripción | Container | Status |
|--------|----------|-----------|-------------|-----------|--------|
| 3000 | grafana | TCP | Dashboards visuales | grafana_trix | ✅ OK |
| 9090 | prometheus | TCP | Time-series metrics DB | prometheus_trix | ✅ OK |
| 3100 | loki | TCP | Log aggregation | loki_trix | ✅ OK |
| 9080 | promtail | TCP | Log collector | promtail_trix | ✅ OK |
| **9081** | **cadvisor** | TCP | Container metrics | cadvisor | 🔧 FIXED |
| 9100 | node_exporter | TCP | Host system metrics | node_exporter | ✅ OK |
| 9121 | redis_exporter | TCP | Redis metrics | redis_exporter | ✅ OK |
| 9187 | postgres_exporter | TCP | PostgreSQL metrics | postgres_exporter | ✅ OK |
| 8929 | gitlab | TCP | Git repo & CI/CD Web | gitlab | ✅ OK |
| 2222 | gitlab | TCP | GitLab SSH | gitlab | ✅ OK |
| 9168 | gitlab | TCP | GitLab Prometheus metrics | gitlab | ✅ OK |

**Dependencias:** redis, postgres, emqx, kong  
**Usuarios:**  
- Grafana: admin / trix1234  
- GitLab: Ver container logs para password inicial  
**Comando:** `trix up-observability`

---

### 🌐 50-edge.yml (Networking & Proxy)
| Puerto | Servicio | Protocolo | Descripción | Container | Status |
|--------|----------|-----------|-------------|-----------|--------|
| 30080 | kong | TCP | API Gateway proxy | kong_db_less | ✅ OK |
| 8100 | kong | TCP | Kong status endpoint | kong_db_less | ✅ OK |
| 80 | caddy | TCP | HTTP reverse proxy | caddy_public | ✅ OK |
| **5353** | **knot** | TCP/UDP | DNS server | knot | 🔧 FIXED |
| - | cloudflare_tunnel | - | Cloudflare Tunnel (host) | cloudflare_tunnel | ℹ️ N/A |

**Dependencias:** ui, micro_ros_agent_hmi, docs builds  
**Network Mode:**  
- cloudflare_tunnel: `network_mode: host` (sin puertos)  
**Comando:** `trix up-edge`

---

### 📚 60-docs-build.yml (Documentación)
| Puerto | Servicio | Protocolo | Descripción | Container | Status |
|--------|----------|-----------|-------------|-----------|--------|
| N/A | hugo_build | - | Hugo static site gen | compose-hugo_build-1 | ✅ OK |
| N/A | doxygen_build | - | C/C++ API docs | compose-doxygen_build-1 | ✅ OK |
| N/A | sphinx_build | - | Python docs | compose-sphinx_build-1 | ✅ OK |

**Tipo:** Build-only containers (`restart: no`)  
**Salida:** Volúmenes montados en Caddy  
**Comando:** `trix up-docs`

---

### 🤖 70-robotics.yml (ROS2 y Sensores)
| Puerto | Servicio | Protocolo | Descripción | Container | Status |
|--------|----------|-----------|-------------|-----------|--------|
| - | robot_encoder | - | ESP32 via USB | ros_relevadores_y_encoder | ✅ OK |
| - | net_sensor_cdmx | - | Network sensor (host) | net_sensor_cdmx | ℹ️ N/A |
| 9080 | trix_agent_orchestrator | TCP | LangGraph AI Agent | trix_agent | ✅ OK |

**Network Mode:**  
- net_sensor_cdmx: `network_mode: host`  
**Dispositivos:**  
- robot_encoder: `/dev/ttyUSB0` (USB serial)  
**Comando:** `trix up-robotics`

---

## 🚨 Conflictos Detectados (CRÍTICO)

### ⚠️ Conflicto #1: Puerto 8080

**Problema:**  
Dos servicios intentan usar el mismo puerto TCP 8080:
- ❌ `springboot_api` (30-apps.yml)
- ❌ `cadvisor` (40-observability.yml) ← **ESTE SE MUEVE**

**Impacto:**  
- Error al iniciar: `Bind for 0.0.0.0:8080 failed: port is already allocated`
- Spring Boot API no arranca O cAdvisor no arranca

**Solución Aplicada:**  
```yaml
# archivo: compose/40-observability.yml
cadvisor:
  ports:
    - "9081:8080"  # ← Puerto externo 9081, interno 8080
```

**Acceso actualizado:**
- Spring Boot API: http://localhost:8080
- cAdvisor: http://localhost:9081

**Actualizar Prometheus:**
```yaml
# archivo: prometheus/prometheus.yml
- job_name: 'cadvisor'
  static_configs:
    - targets: ['cadvisor:8080']  # ← NO cambiar (puerto interno)
```

---

### ⚠️ Conflicto #2: Puerto 1053

**Problema:**  
Puerto comúnmente ocupado por servicios del sistema o antivirus:
- ❌ `knot` (50-edge.yml) intenta usar 1053/TCP+UDP
- ❌ Windows puede tener servicios en este puerto

**Impacto:**  
- Error al iniciar: `Bind for 0.0.0.0:1053 failed: port is already allocated`
- DNS local no funciona

**Solución Aplicada:**  
```yaml
# archivo: compose/50-edge.yml
knot:
  ports:
    - "5353:1053/udp"  # ← Puerto DNS alternativo común
    - "5353:1053/tcp"
```

**Acceso actualizado:**
- Knot DNS: `dig @localhost -p 5353 example.test`

---

## ✅ Soluciones Aplicadas

### 1. Actualizar compose/40-observability.yml

```yaml
services:
  cadvisor:
    image: gcr.io/cadvisor/cadvisor:v0.49.1
    container_name: cadvisor
    networks: [trix-net]
    ports:
      - "9081:8080"  # ← CAMBIAR AQUÍ (era 8080:8080)
    restart: unless-stopped
    volumes:
      - /:/rootfs:ro
      - /var/run:/var/run:ro
      - /sys:/sys:ro
      - /var/lib/docker/:/var/lib/docker:ro
      - /dev/disk/:/dev/disk:ro
    privileged: true
    devices:
      - /dev/kmsg
```

### 2. Actualizar compose/50-edge.yml

```yaml
services:
  knot:
    build:
      context: ../knot_dns
    container_name: knot
    networks: [trix-net]
    ports:
      - "5353:1053/udp"  # ← CAMBIAR AQUÍ (era 1053:1053)
      - "5353:1053/tcp"  # ← CAMBIAR AQUÍ (era 1053:1053)
    volumes:
      - ../knot_dns/knot.conf:/etc/knot/knot.conf:ro
      - ../knot_dns/zones:/zones:ro
      - knot_data:/var/lib/knot
    cap_add: [NET_BIND_SERVICE]
    healthcheck:
      test: ["CMD", "kdig", "@127.0.0.1", "-p", "1053", "example.test", "SOA"]
      interval: 30s
      timeout: 5s
      retries: 5
    restart: unless-stopped
```

### 3. Actualizar prometheus/prometheus.yml

```yaml
global:
  scrape_interval: 15s
  evaluation_interval: 15s

scrape_configs:
  - job_name: 'prometheus'
    static_configs:
      - targets: ['localhost:9090']

  - job_name: 'loki'
    static_configs:
      - targets: ['loki:3100']

  - job_name: 'promtail'
    static_configs:
      - targets: ['promtail:9080']

  - job_name: 'grafana'
    static_configs:
      - targets: ['grafana:3000']

  - job_name: 'redis'
    static_configs:
      - targets: ['redis_exporter:9121']

  - job_name: 'postgres'
    static_configs:
      - targets: ['postgres_exporter:9187']

  - job_name: 'node'
    static_configs:
      - targets: ['node_exporter:9100']

  - job_name: 'cadvisor'
    static_configs:
      - targets: ['cadvisor:8080']  # ← Puerto INTERNO (no cambiar)

  - job_name: 'gitlab'
    static_configs:
      - targets: ['gitlab:9168']

  - job_name: 'kong'
    static_configs:
      - targets: ['kong:8100']

  - job_name: 'emqx'
    static_configs:
      - targets: ['emqx:18083']
```

---

## 📊 Resumen de Puertos por Rango

| Rango | Propósito | Servicios | Total |
|-------|-----------|-----------|-------|
| **1000-1999** | Messaging | MQTT (1883), Node-RED (1880) | 2 |
| **2000-2999** | DevOps | GitLab SSH (2222) | 1 |
| **3000-3999** | Dashboards | Grafana (3000), Loki (3100) | 2 |
| **5000-5999** | Apps Frontend | Flask UI (5000, 5002/udp), PostgreSQL (5432), Knot DNS (**5353**) | 4 |
| **6000-6999** | Business Apps | Redis (6379), Trix ERP (6800) | 2 |
| **7000-7999** | Utilities | Git (7011) | 1 |
| **8000-8999** | APIs/Web | Spring (8080), EMQX WS (8083), Trixweb (8030), GitLab (8929), Kong Status (8100), Caddy (80) | 6 |
| **9000-9999** | Monitoring | Prometheus (9090), Exporters (9100, 9121, 9187, **9081**), PgAdmin (9070), Promtail (9080), Trix Agent (9080), Micro-ROS (9999/udp), GitLab metrics (9168) | 10 |
| **18000+** | Alt Ports | EMQX Dashboard (18083) | 1 |
| **30000+** | Public | Kong Gateway (30080) | 1 |

**Total:** 30 puertos asignados

---

## 🎯 Lista Completa Ordenada

### Puertos TCP
```
80      caddy (HTTP)
1880    node_red
1883    emqx (MQTT)
2222    gitlab (SSH)
3000    grafana
3100    loki
5000    ui (Flask)
5353    knot DNS (TCP) ← FIXED
5432    postgres
6379    redis
6800    trix-erp
7011    git_observer
8030    trixweb
8080    springboot_api
8083    emqx (WebSocket)
8100    kong (status)
8929    gitlab (web)
9070    pgadmin
9080    promtail, trix_agent
9081    cadvisor ← FIXED
9090    prometheus
9100    node_exporter
9121    redis_exporter
9168    gitlab (metrics)
9187    postgres_exporter
18083   emqx (dashboard)
30080   kong (proxy)
```

### Puertos UDP
```
5002    ui (Flask data)
5353    knot DNS (UDP) ← FIXED
9999    micro_ros_agent
```

---

## 🛠️ Comandos de Validación

### Validar todos los puertos
```cmd
trix validate-ports
```

### Ver puertos ocupados en el sistema
```cmd
netstat -ano | findstr LISTENING
```

### Ver puertos específicos
```cmd
netstat -ano | findstr :8080
netstat -ano | findstr :1053
```

### Ver puertos de contenedores Docker
```cmd
docker ps --format "table {{.Names}}\t{{.Ports}}"
```

### Limpiar conflictos automáticamente
```cmd
trix clean-all
```

### Matar proceso en puerto específico
```cmd
REM Encontrar PID
netstat -ano | findstr :8080

REM Matar proceso (reemplaza XXXX con el PID)
taskkill /PID XXXX /F
```

---

## 📝 Checklist Pre-Deploy

### ✅ Antes de iniciar servicios:

- [ ] **Aplicar cambios en composes**
  - [ ] `compose/40-observability.yml`: cAdvisor puerto 8080 → 9081
  - [ ] `compose/50-edge.yml`: Knot puerto 1053 → 5353
  
- [ ] **Actualizar configuraciones**
  - [ ] `prometheus/prometheus.yml`: Verificar targets
  - [ ] `grafana/provisioning/datasources/datasources.yml`: URLs correctas
  
- [ ] **Validar entorno**
  - [ ] Docker Desktop está corriendo
  - [ ] Archivo `.env` existe con variables necesarias
  - [ ] Ejecutar `trix validate-ports`
  
- [ ] **Limpiar estado anterior**
  - [ ] Ejecutar `trix clean-all`
  - [ ] Verificar que puertos 8080 y 1053 estén libres
  
- [ ] **Iniciar servicios**
  - [ ] Ejecutar `trix up-sequential` (recomendado)
  - [ ] O `trix up` para inicio rápido

### ✅ Después de iniciar:

- [ ] **Verificar servicios core**
  - [ ] Redis: `docker logs redis_trix --tail 20`
  - [ ] PostgreSQL: `docker logs postgres_trix --tail 20`
  - [ ] EMQX: http://localhost:18083 (admin/trix1234)
  
- [ ] **Verificar aplicaciones**
  - [ ] Flask UI: http://localhost:5000
  - [ ] Node-RED: http://localhost:1880
  - [ ] Spring API: http://localhost:8080/actuator/health
  
- [ ] **Verificar observabilidad**
  - [ ] Grafana: http://localhost:3000 (admin/trix1234)
  - [ ] Prometheus: http://localhost:9090/targets
  - [ ] cAdvisor: http://localhost:9081 ← **NUEVO PUERTO**
  - [ ] Loki: `curl http://localhost:3100/ready`
  
- [ ] **Verificar edge**
  - [ ] Kong: http://localhost:30080
  - [ ] Caddy: http://localhost:80
  - [ ] Knot DNS: `nslookup -port=5353 example.test localhost`

---

## 🔒 Puertos Reservados (NO USAR)

Estos puertos están reservados por el sistema o son comúnmente usados:

- **21**: FTP
- **22**: SSH del host
- **25**: SMTP
- **53**: DNS del sistema (usa 5353)
- **443**: HTTPS (Caddy en dev usa 80, en prod 443)
- **445**: SMB/CIFS
- **3306**: MySQL (si se planea agregar)
- **3389**: RDP de Windows
- **5000**: Windows reservado (pero Docker puede usar)
- **5432**: PostgreSQL (ya en uso)
- **8080**: Muy común, evitar para nuevos servicios
- **27017**: MongoDB (si se planea agregar)

---

## 📞 Notas Técnicas

### Network Modes Especiales

**Services con `network_mode: host`:**
- `cloudflare_tunnel`: Comparte red del host, sin puertos expuestos
- `net_sensor_cdmx`: Necesita acceso directo a la red para ARP scanning

**Implicaciones:**
- No usan `ports:` en compose
- Acceden directamente a puertos del host
- No están en la red `trix-net`

### Servicios Build-Only

**Containers con `restart: no`:**
- `hugo_build`
- `doxygen_build`
- `sphinx_build`

**Características:**
- Se ejecutan una vez y terminan
- Generan archivos en volúmenes
- Caddy sirve los archivos generados
- No necesitan puertos

### Dependencias Críticas

**Orden de inicio recomendado:**
1. Foundation (Redis)
2. Data (PostgreSQL)
3. Messaging (EMQX)
4. Apps (UI, Node-RED, Spring, ERP)
5. Observability (Grafana, Prometheus, Loki)
6. Edge (Kong, Caddy, Knot)
7. Robotics (ROS2)
8. Docs (Hugo, Doxygen, Sphinx)

**Uso:** `trix up-sequential` sigue este orden automáticamente

### Healthchecks Implementados

Servicios con healthcheck activo:
- ✅ redis
- ✅ postgres
- ✅ emqx
- ✅ caddy
- ✅ knot
- ✅ gitlab
- ✅ grafana
- ✅ prometheus
- ✅ loki

---

## 🆘 Troubleshooting

### Error: "port is already allocated"

```cmd
REM 1. Identificar proceso
netstat -ano | findstr :PUERTO

REM 2. Ver qué proceso es
tasklist | findstr PID

REM 3. Matar proceso
taskkill /PID XXXX /F

REM 4. Reintentar
trix up-sequential
```

### Contenedores no inician

```cmd
REM Ver logs
docker logs NOMBRE_CONTENEDOR --tail 50

REM Ver todos los servicios
trix ps

REM Reiniciar servicio específico
docker restart NOMBRE_CONTENEDOR
```

### Puerto ocupado pero sin proceso visible

```cmd
REM Puede ser un contenedor Docker anterior
docker ps -a | findstr PUERTO

REM Eliminar contenedor
docker rm -f NOMBRE_CONTENEDOR

REM Limpiar todo
trix clean-all
```

---

## 📚 Referencias

- **Documentación Docker:** https://docs.docker.com/
- **EMQX:** https://www.emqx.io/docs/
- **Grafana:** https://grafana.com/docs/
- **Prometheus:** https://prometheus.io/docs/
- **Kong:** https://docs.konghq.com/
- **Node-RED:** https://nodered.org/docs/

---

## 📜 Changelog

| Fecha | Cambio | Autor |
|-------|--------|-------|
| 2025-01-15 | Creación del documento | DevOps Team |
| 2025-01-15 | Fix: cAdvisor 8080→9081 | DevOps Team |
| 2025-01-15 | Fix: Knot DNS 1053→5353 | DevOps Team |

---

**Documento mantenido por:** TRIX DevOps Team  
**Última revisión:** 2025-01-15  
**Versión:** 1.0.0