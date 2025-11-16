# app/ros_monitor.py
import json
import time
import threading
from typing import Dict, Any, List
from flask import Blueprint, jsonify, Response, current_app

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

# ─────────────────────────────────────────────────────────────
# Blueprint y setup
# ─────────────────────────────────────────────────────────────
ros_bp = Blueprint("ros_monitor", __name__)

# Guardamos un único Node y Executor para todo el proceso
_ctx_lock = threading.Lock()
_ctx: Dict[str, Any] = {"node": None, "executor": None, "ready": False}

class _GraphNode(Node):
    def __init__(self):
        super().__init__("trix_graph_monitor")

def _ensure_node() -> Node:
    """
    Crea (una vez) un Node y un Executor para consultas ROS2.
    """
    with _ctx_lock:
        if not _ctx["ready"]:
            if not rclpy.utilities.ok():
                rclpy.init(args=None)
            node = _GraphNode()
            exec_ = SingleThreadedExecutor()
            exec_.add_node(node)
            _ctx.update(node=node, executor=exec_, ready=True)
    return _ctx["node"]

def _spin_once(timeout_sec: float = 0.01):
    """
    Atiende callbacks mínimos del executor (no bloqueante).
    """
    ex = _ctx.get("executor")
    if ex:
        try:
            ex.spin_once(timeout_sec=timeout_sec)
        except Exception:
            # Evitar que un error de spin tumbe el endpoint
            pass

def _collect_graph() -> Dict[str, Any]:
    """
    Construye un grafo básico: lista de nodos, lista de tópicos con tipo,
    publishers/subscribers y un hint de QoS (reliability).
    """
    n = _ensure_node()
    _spin_once(0.01)

    # Nodos
    nodes: List[Dict[str, Any]] = []
    try:
        for name, ns in n.get_node_names_and_namespaces():
            nodes.append({"name": name, "namespace": ns, "publishes": [], "subscribes": []})
    except Exception as e:
        current_app.logger.warning("get_node_names_and_namespaces failed: %s", e)

    # Tópicos
    topics: List[Dict[str, Any]] = []
    try:
        topics_info = n.get_topic_names_and_types()
        for tname, ttypes in topics_info:
            ttype = (ttypes[0] if ttypes else "") or ""
            pub_infos = n.get_publishers_info_by_topic(tname)
            sub_infos = n.get_subscriptions_info_by_topic(tname)

            pubs = [i.node_name for i in pub_infos]
            subs = [i.node_name for i in sub_infos]

            # QoS (resumen simple por tópico; si hay algún BEST_EFFORT lo marcamos)
            qos_list = []
            try:
                any_be = False
                for i in list(pub_infos) + list(sub_infos):
                    qp = getattr(i, "qos_profile", None)
                    rel = getattr(qp, "reliability", None)
                    if hasattr(rel, "name") and rel.name == "BEST_EFFORT":
                        any_be = True
                        break
                qos_list.append({"reliability": "BEST_EFFORT" if any_be else "RELIABLE"})
            except Exception:
                pass

            topics.append({
                "name": tname,
                "type": ttype,
                "publishers": pubs,
                "subscribers": subs,
                "qos": qos_list
            })
    except Exception as e:
        current_app.logger.warning("get_topic_names_and_types failed: %s", e)

    # Completar pubs/subs por nodo
    pubs_map: Dict[str, List[str]] = {}
    subs_map: Dict[str, List[str]] = {}
    for ti in topics:
        for p in ti["publishers"]:
            pubs_map.setdefault(p, []).append(ti["name"])
        for s in ti["subscribers"]:
            subs_map.setdefault(s, []).append(ti["name"])

    for nd in nodes:
        nd["publishes"]  = pubs_map.get(nd["name"], [])
        nd["subscribes"] = subs_map.get(nd["name"], [])

    return {"nodes": nodes, "topics": topics, "ts": time.time()}

# ─────────────────────────────────────────────────────────────
# Rutas HTTP
# ─────────────────────────────────────────────────────────────

@ros_bp.route("/ros/graph", methods=["GET"])
def ros_graph():
    try:
        data = _collect_graph()
        return jsonify(data)
    except Exception as e:
        current_app.logger.exception("ros_graph error")
        return jsonify({"status": "error", "message": str(e)}), 500

@ros_bp.route("/ros/graph/stream", methods=["GET"])
def ros_graph_stream():
    """
    SSE opcional. Si el frontend no lo usa, tu UI ya hace polling /ros/graph.
    """
    def gen():
        while True:
            try:
                payload = json.dumps(_collect_graph(), separators=(",", ":"))
                yield f"event: graph\ndata: {payload}\n\n"
                time.sleep(2.0)
            except GeneratorExit:
                break
            except Exception as e:
                current_app.logger.warning("SSE error: %s", e)
                time.sleep(2.0)
    return Response(gen(), mimetype="text/event-stream")

# ─────────────────────────────────────────────────────────────
# Registro/teardown
# ─────────────────────────────────────────────────────────────

def init_ros_monitor(app):
    """
    Llama a esto desde tu app.py: init_ros_monitor(app)
    """
    app.register_blueprint(ros_bp)

    @app.teardown_appcontext
    def _shutdown_ctx(exc):
        # No apagamos rclpy aquí si compartes ROS con otros módulos.
        # Si alguna vez quieres apagar, hazlo de forma explícita al detener el proceso.
        return None

