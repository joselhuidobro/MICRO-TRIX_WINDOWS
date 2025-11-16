# config.py
import os

# ROS / UDP
ROS_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "0"))
UDP_PORT      = int(os.environ.get("UDP_PORT", "5002"))

# MODBUS (nuevos nombres)
MODBUS_HOST    = os.environ.get("MODBUS_HOST", "192.168.1.100")
MODBUS_PORT    = int(os.environ.get("MODBUS_PORT", "502"))
MODBUS_UNIT    = int(os.environ.get("MODBUS_UNIT", "1"))
MODBUS_ENABLED = os.environ.get("MODBUS_ENABLED", "1") not in ("0", "false", "False", "")

# Alias de compatibilidad con código antiguo:
hmi_ip    = MODBUS_HOST
port      = MODBUS_PORT
device_id = MODBUS_UNIT

# Solo usado por el JS como referencia
poll_interval = 0.1

