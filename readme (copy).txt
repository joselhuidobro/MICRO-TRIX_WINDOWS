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

Tu MCU está en la misma LAN que tu host (Wi-Fi 192.168.0.0/24).

Le dijimos al MCU que hable con tu host (192.168.0.203) por UDP 8888.

En tu host pusimos reglas DNAT/MASQUERADE que reenrutan ese tráfico a Minikube:

192.168.0.203:8888/udp → 192.168.49.2:30888/udp (NodePort del micro-ROS Agent).

Así, el host hace de puente entre la LAN y la red interna de Minikube (192.168.49.0/24).
(La IP 192.168.49.2 de Minikube no es alcanzable directamente desde tu MCU).


.................
prender rele desde terminal 



ros2 topic pub --once /relevadores std_msgs/msg/String "data: relay1_on"
ros2 topic pub --once --qos-reliability best_effort /servo_setpoint_deg std_msgs/msg/Float32 "{data: 90.0}"

para probar el pplaneador 
ros2 service call /plan std_srvs/srv/Trigger {}

probar end point de API . pedixr por HTTP
 curl -i http://localhost:8080/hello


curl -i http://127.0.0.1/_apg
curl -i http://127.0.0.1/                 # HTML de tu app
curl -I http://127.0.0.1/static/css/styles.css
curl -i http://127.0.0.1/api/health
curl -i http://127.0.0.1/flows/
# SSE (queda “colgado” si hay stream)
curl -i -N http://127.0.0.1/udp/devices/stream
# WebSocket handshake
curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" \
  "http://127.0.0.1/socket.io/?EIO=4&transport=websocket"
# MQTT WS handshake
curl -i -N -H "Connection: Upgrade" -H "Upgrade: websocket" \
  http://127.0.0.1/mqtt/

GAZEBO --------------

docker exec -it gazebo_sim bash -lc '
  source /usr/share/gazebo/setup.sh
  export DISPLAY='"$DISPLAY"'
  export QT_QPA_PLATFORM=xcb
  export QT_X11_NO_MITSHM=1
  export LIBGL_ALWAYS_SOFTWARE=1
  export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
  export OGRE_RTT_MODE=Copy
  # Asegura rutas de recursos (shaders, media, worlds)
  export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH}
  export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}
  # Conéctate al master que ya corre en el propio contenedor
  export GAZEBO_MASTER_URI=http://127.0.0.1:11345
  gzclient --verbose
aulting to '/tmp/runtime-root'
----------------------------------------------------------
docker exec -it gazebo_sim bash -lc 'gz topic -l | head'
----------------

 para abrir gazebo correr en terminal local donde esta el compose: gazebo-gui.sh 
------------------------

cOMANDOS idf
idf.py fullclean
idf.py build
idf.py -p /dev/ttyUSB0 flash
idf.py -p /dev/ttyUSB0 monitor
idf.py -p /dev/ttyUSB0 monitor	

sudo docker stop $(sudo docker ps -aq) 
sudo docker stop robot
sudo docker rm robot
sudo docker rmi ubuntu:robot # Or whatever your image name is

# Elimina todos los contenedores y volúmenes
sudo docker rm -f $(docker ps -aq) 2>/dev/null
sudo docker volume rm $(docker volume ls -q) 2>/dev/null

# Limpia el sistema Docker
sudo docker system prune -af
sudo docker system prune -a --volumes


sudo docker-compose up --build -d
sudo docker exec -it robot bash

grep "ESP_WIFI_" build/config/sdkconfig.h
 ls components | grep micro_ros
find components/micro_ros_espidf_component -type f -name '*transport*.h' | head
 idf.py reconfigure | sed -n '/-- Components:/,/--/p' | tail -n +2 | tr -d ','
Compiler supported targets: xtensa-esp32-elf


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

uidobro@gmail.com"
Generating public/private ed25519 key pair.
Enter file in which to save the key (/home/jose/.ssh/id_ed25519): 
Enter passphrase (empty for no passphrase): 
Enter same passphrase again: 
Your identification has been saved in /home/jose/.ssh/id_ed25519.
Your public key has been saved in /home/jose/.ssh/id_ed25519.pub.
The key fingerprint is:
SHA256:e4fpxYS/zrZ/1dueSAAsfaoyhqPClSTeRfOq/nOCuX4 jhuidobro@gmail.com
The key's randomart image is:
+--[ED25519 256]--+
|                 |
|     o  o        |
|    . o. + .     |
| . . . .. +.     |
|. + o . S....   .|
| . +..  .. *.   o|
|. .o++ .. + =.  +|
|...+oEo. o +o...+|
|..o++.+   .o=+.+.|
+----[SHA256]-----+
jose@jose-desktop:~/Documents/2025master-DRONespui$ eval "$(ssh-agent -s)"


ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIOBKp/JP16mTm8aCFe/DnGk78Dt+tUTNSt/Ox9L2u2lw jhuidobro@gmail.com

