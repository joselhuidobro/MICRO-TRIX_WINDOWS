# Imagen oficial Node-RED (Debian)
FROM nodered/node-red:3.1-debian

# Preinstala módulos útiles
# - Dashboard: UI rápida
# - Modbus: conexión opcional a HMI/PLC por TCP
# Puedes agregar otros: node-red-contrib-ros2 (si lo usas con rosbridge), etc.
RUN npm install --unsafe-perm --no-update-notifier --no-fund --only=production \
    node-red-dashboard \
    node-red-contrib-modbus

