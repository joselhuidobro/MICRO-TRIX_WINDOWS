# ros_utils.py  (versión robusta: QoS fiable + tópicos configurables + debug)
import os, re, time, queue, threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String as RosString
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

TELEM_RE = re.compile(r"Degrees: (-?\d+(?:\.\d+)?), Pulses: (-?\d+)")

def parse_telem_string(s: str):
    s = s.strip()
    m = TELEM_RE.search(s)
    if not m:
        return None
    degrees, pulses = m.groups()
    return {"degrees": float(degrees), "pulses": int(pulses)}

def _normalize_topic(name: str) -> str:
    # Evita usar "/" inicial para no forzar absoluto y facilitar matching
    return name.lstrip("/")

class TelemetrySubscriber(Node):
    def __init__(
        self,
        publish_update,
        command_queue,
        telemetry_topic='RELAY_telemetry_11B384',
        setpoint_topic=None,
        relay_topic=None,
        debug=False,
    ):
        super().__init__('ui_telemetry_subscriber')
        self.publish_update   = publish_update
        self.command_queue    = command_queue
        self.debug            = debug
        self.current_degrees  = 0.0

        # QoS: telemetría suele venir BEST_EFFORT, pero publishers a uC deben ser RELIABLE
        qos_sub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        qos_pub_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Tópicos (permite override por env)
        self.telemetry_topic = _normalize_topic(telemetry_topic)
        self.setpoint_topic  = _normalize_topic(setpoint_topic or os.getenv('SETPOINT_TOPIC', 'servo_setpoint_deg'))
        self.relay_topic     = _normalize_topic(relay_topic    or os.getenv('RELAY_TOPIC',    'relevadores'))

        # Subs / Pubs
        self.create_subscription(RosString, self.telemetry_topic, self.cb, qos_sub)
        self.relay_publisher = self.create_publisher(RosString, self.relay_topic, qos_pub_reliable)
        self.setpoint_pub    = self.create_publisher(Float32,   self.setpoint_topic, qos_pub_reliable)

        # --- Rutina bajo demanda (configurable por ENV) ---
        self.plan_angles = [float(x) for x in os.getenv("ANGLES", "0,45,-45,0, 360, 90,0,600,0").split(",") if x.strip()]
        self.plan_dt = float(os.getenv("DT", "1.5"))
        # Servicio absoluto para evitar líos de namespace
        self.create_service(Trigger, "/plan", self._srv_plan_cb)

        if self.debug:
            self.get_logger().info(f"[ROS] Sub telem: {self.telemetry_topic}")
            self.get_logger().info(f"[ROS] Pub setpt: {self.setpoint_topic} (RELIABLE)")
            self.get_logger().info(f"[ROS] Pub relay: {self.relay_topic} (RELIABLE)")
            self.get_logger().info(f"[ROS] Servicio listo: /plan (Trigger)  ANGLES={self.plan_angles}  DT={self.plan_dt}s")

    def cb(self, msg: RosString):
        if self.debug:
            self.get_logger().info(f"[ROS] RX: {msg.data}")
        data = parse_telem_string(msg.data)
        if not data:
            if self.debug:
                self.get_logger().warn(f"[ROS] Parse FAIL: {msg.data}")
            return
        self.current_degrees = float(data.get("degrees", self.current_degrees))
        if self.debug:
            self.get_logger().info(f"[ROS] parsed {data} | current_deg={self.current_degrees:.2f}")
        self.publish_update(data)

    # ---------- Helpers de publicación ----------
    def _publish_setpoint(self, value: float, burst: int = 1):
        m = Float32()
        for _ in range(max(1, int(burst))):
            m.data = float(value)
            self.setpoint_pub.publish(m)
            time.sleep(0.02)  # pequeño gap ayuda en micro-ROS
        self.get_logger().info(f"[ROS] Setpoint {value} x{burst} → {self.setpoint_topic}")

    def _publish_relay(self, index: int, state: bool):
        relay_num = int(index) + 1
        cmd = f"relay{relay_num}_{'on' if state else 'off'}"
        self.relay_publisher.publish(RosString(data=cmd))
        self.get_logger().info(f"[ROS] Relay cmd: {cmd} → {self.relay_topic}")

    def process_commands(self):
        while True:
            try:
                item = self.command_queue.get_nowait()
            except queue.Empty:
                break

        # Normaliza formatos
            cmd, payload = None, None
            if isinstance(item, tuple) and len(item) == 2 and isinstance(item[0], str):
                cmd, payload = item
            elif isinstance(item, tuple) and len(item) == 2 and isinstance(item[0], int):
                self._publish_relay(item[0], bool(item[1]))
                continue
            elif isinstance(item, dict):
                cmd = item.get("cmd"); payload = item
            else:
                self.get_logger().warn(f"[ROS] Formato de comando desconocido: {item!r}")
                continue

            # Ejecuta comando
            if cmd == "relay":
                if isinstance(payload, dict):
                    idx = int(payload.get("index", 0))
                    val = bool(payload.get("value", False))
                    self._publish_relay(idx, val)
                else:
                    self.get_logger().warn(f"[ROS] Payload inválido para relay: {payload!r}")

            elif cmd == "setpoint":
                val  = float(payload.get("value"))
                brst = int(payload.get("burst", 5))
                self._publish_setpoint(val, brst)

            elif cmd == "servo":
                step      = float(payload.get("step", 10.0))
                direction = int(payload.get("direction", 1))
                brst      = int(payload.get("burst", 10))
                new_sp    = self.current_degrees + (direction * step)
                self._publish_setpoint(new_sp, brst)

            else:
                self.get_logger().warn(f"[ROS] Cmd desconocido '{cmd}' payload={payload!r}")

    # ---------- Servicio: /plan ----------
    def _run_plan(self):
        for deg in self.plan_angles:
            # Publica con tu publisher RELIABLE (usa tu helper)
            self._publish_setpoint(deg, burst=1)
            if self.debug:
                self.get_logger().info(f"[ROS] rutina -> {deg} deg")
            time.sleep(self.plan_dt)
        self.get_logger().info("[ROS] rutina terminada.")

    def _srv_plan_cb(self, req, res):
        # No bloquear el callback del servicio
        threading.Thread(target=self._run_plan, daemon=True).start()
        res.success = True
        res.message = f"Publicando {len(self.plan_angles)} setpoints en {self.setpoint_topic} (dt={self.plan_dt}s)"
        self.get_logger().info("[ROS] /plan solicitado -> arrancando rutina")
        return res

def ros_thread(publish_update, ros_domain_id, command_queue,
               telemetry_topic='RELAY_telemetry_11B384', debug=False):
    os.environ["ROS_DOMAIN_ID"] = str(ros_domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "0"

    dbg_env = os.getenv("ROS_UI_DEBUG", "0").strip()
    debug = debug or (dbg_env in ("1", "true", "True", "yes", "YES"))

    if debug:
        print(f"[ROS] DOMAIN_ID={ros_domain_id}")
        print("[ROS] init()…")
    try:
        rclpy.init()
        if debug: print("[ROS] OK init()")
    except Exception as e:
        print(f"[ROS] rclpy.init() falló: {e}")
        return

    node = TelemetrySubscriber(
        publish_update, command_queue,
        telemetry_topic=telemetry_topic,
        debug=debug
    )

    last_dbg = time.time()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            node.process_commands()
            if debug and (time.time() - last_dbg) > 3.0:
                pubs_sp = node.count_publishers(node.setpoint_topic)
                subs_sp = node.count_subscribers(node.setpoint_topic)
                pubs_rl = node.count_publishers(node.relay_topic)
                subs_rl = node.count_subscribers(node.relay_topic)
                node.get_logger().info(
                    f"[ROS DEBUG] {node.setpoint_topic}: pubs={pubs_sp}, subs={subs_sp} | "
                    f"{node.relay_topic}: pubs={pubs_rl}, subs_rl={subs_rl}"
                )
                last_dbg = time.time()
    except Exception as e:
        print(f"[ROS] Loop error: {e}")
    finally:
        try: node.destroy_node()
        except Exception: pass
        try: rclpy.shutdown()
        except Exception: pass

