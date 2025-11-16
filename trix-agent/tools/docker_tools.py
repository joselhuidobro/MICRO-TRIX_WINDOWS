import docker
import os
from langchain.tools import tool

def get_docker_client():
    """Auto-detecta Docker en Windows (TCP) o Linux (socket)"""
    # Windows: usa TCP
    if os.name == 'nt' or "DOCKER_HOST" in os.environ:
        return docker.DockerClient(base_url="tcp://host.docker.internal:2375")
    
    # Linux: usa socket
    return docker.DockerClient(base_url="unix:///var/run/docker.sock")

@tool
def restart_robot_container(device_id: str) -> dict:
    """Reinicia container robot_encoder_{device_id}"""
    try:
        client = get_docker_client()
        # Docker Compose en Windows crea nombres así: compose-robot_encoder-1
        container_name = f"ros2_relay_hmi_modbus-main-robot_encoder-1"
        container = client.containers.get(container_name)
        container.restart(timeout=10)
        return {"success": True, "message": f"✅ {container_name} reiniciado"}
    except Exception as e:
        return {"success": False, "error": str(e)}

@tool
def check_container_status(device_id: str) -> bool:
    """Devuelve True si está corriendo"""
    try:
        client = get_docker_client()
        container = client.containers.get(f"ros2_relay_hmi_modbus-main-robot_encoder-1")
        return container.status == "running"
    except:
        return False