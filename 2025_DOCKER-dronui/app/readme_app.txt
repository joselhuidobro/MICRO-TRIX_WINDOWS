app.py (versión completa)

Qué es: Servidor Flask que integra ROS 2 + UDP + Modbus/HMI + ESP-IDF (Docker), con SSE y radar UDP. Centraliza telemetría/estados y expone endpoints de UI y control.

Servicios de fondo:

start_udp_listener → publica al bus (publish_update) y al radar (_src="UDP").

ros_thread → consume ros_publish_q, publica al bus/radar (_src="ROS").

start_modbus_poller → lee Modbus periódicamente y publica cambios.

udp_radar.start() → descubrimiento activo (UP/DOWN) y logging.

Rutas (agrupadas):

UI: GET / (index con lw0..2, hmi_online, degrees/pulses).

Telemetría: GET /telemetry (JSON), GET /orientation (SSE + heartbeat).

Radar UDP: GET /udp/devices (JSON), GET /udp/devices/stream (SSE).

ESP-IDF (Docker ‘robot’): POST /build_idf, /flash_esp32, /idf_monitor.

Modbus/Relés: GET /data, POST /toggle_lw0|1|2.

Movimiento: POST /servo/right (/api/servo/right), POST /servo/left (/api/servo/left).

Debug: POST /debug/ping, GET /debug/queue-size.

Robustez: safe_get_modbus/safe_toggle, SSE con cabeceras anti-buffering, logging por request (ID, latencia, cuerpo truncado).

Config clave: ROS_DOMAIN_ID, UDP_PORT, UDP_OFFLINE_TIMEOUT_S, UDP_FOCUS_PROBE_INTERVAL_S, UDP_PROBE_WINDOW_S, UDP_DISCOVERY_LOG.

config.py

Lee ROS_DOMAIN_ID (def. 10) y UDP_PORT (def. 5002).

Define Modbus/HMI: hmi_ip, port(502), device_id(1), poll_interval(0.1).

Sugerencia: renombrar a MODBUS_PORT y permitir override por env.

json_utils.py

fast_dumps(obj): usa orjson (si está) para JSON rápido; fallback a json estándar (compacto). Ideal para SSE.

Variante ligera (UDP+ROS → SSE)

Qué es: Flask + SSE con telemetría que llega por UDP binario y por ROS 2 (drone_telemetry).

Claves:

UDP: desempaqueta <3f4hI> (con t_us) o <3f4h>; calcula lat_ms; publica con _publish_update.

ROS 2: parsea Ang: P/R/Y | M: m1 m2 m3 m4 y publica a SSE.

Endpoints: /, /orientation, y acciones Docker (/build_idf, /flash_esp32, /idf_monitor), más /get_code.

Arranque: hilos udp_listener y ros_thread; Flask en 0.0.0.0:5500.

main.py (esperado)

Punto de entrada para Gunicorn/producción. Importa app y garantiza el arranque de hilos/servicios (UDP, ROS, Modbus, radar) fuera de __main__ si hace falta.

modbus_functions.py (esperado)

get_modbus_data() → lee estado HMI (lw0.., hmi_online, etc.).

toggle_lw(idx) → alterna coil/registro y devuelve {status, new_value,...}.

perform_modbus_scan() → (opcional) descubre dispositivos/IDs activos.

modbus_service.py (esperado)

start_modbus_poller(get_fn, publish_fn, interval) → bucle que publica diffs de Modbus al bus; maneja errores/hmi_online.

ros_telem.py (o ros_telem.py.py) (esperado)

Nodo ROS 2: suscribe telemetría y publica a bus/UI.

Funciones para publicar setpoints/relés y un spin/hilo que procesa la cola ros_publish_q.
