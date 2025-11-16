# main.py
import threading
from udp import udp_listener
from ros import ros_thread
from app import app

if __name__ == "__main__":
    # Hilos: UDP + ROS
    threading.Thread(target=udp_listener, daemon=True).start()
    threading.Thread(target=ros_thread, daemon=True).start()

    app.run(host="0.0.0.0", port=5500, threaded=True)
