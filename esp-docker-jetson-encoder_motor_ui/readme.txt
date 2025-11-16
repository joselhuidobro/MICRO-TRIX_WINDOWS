/*───────────────────────────────────
 *  ████████╗██████╗ ██╗██╗  ██╗    
 *  ╚══██╔══╝██╔══██╗██║╚██╗██╔╝   
 *     ██║   ██████╔╝██║ ╚███╔╝     
 *     ██║   ██╔══██╗██║ ██╔██╗     
 *     ██║   ██║  ██║██║██╔╝ ██╗    
 *     ╚═╝   ╚═╝  ╚═╝╚═╝╚═╝  ╚═╝   
 *──────────────────────────────────
 *  TRIX PID BLOCK  ▸  Resumen de integración

 *      • Target : ESP32-WROOM-32 • ESP-IDF v5.1.6-680-g7bc49c1227 (release/5.1)
 *  ◼ Docker Image
 *      • Base   : ros:humble-ros-base-jammy (Ubuntu 22.04 + ROS 2 Humble)
 *      • Tag    : esp-dev (Built by the project Dockerfile)


PARA ESP32 ESPRESSIFF DEL MODULE

Este Dockerfile crea una imagen basada en ros:humble-ros-base-jammy (Ubuntu 22.04 con ROS 2 Humble ya instalado) 
y la amplía para ofrecer un entorno de desarrollo integrado para proyectos que combinan ROS 2 y firmware ESP32

Este Dockerfile crea un entorno de desarrollo
ESP-IDF v5.1.6-680-g7bc49c1227

✅ Versión de ESP-IDF: release/v5.1 (correcta para desarrollo actual).
✅ Dependencias instaladas: Todas las necesarias (git, cmake, ninja, python3, gcc-arm-none-eabi, etc.).
✅ Estructura del proyecto: Copia tu código en /esp2024 y compila con idf.py build.
✅ Componente         espressif__pid_ctrl version v0.1.1
    Componente espressif__mpu6050 1.2.0


/esp2024
├── components/          # Custom and third-party ESP-IDF components
│   ├── arduino-esp32/   # Arduino as an ESP-IDF component (for Arduino libraries)
│   │   ├── CMakeLists.txt
│   │   ├── idf_component.yml
│   │   └── libraries/   # Arduino libraries (e.g., ArduinoOTA)
│   │       └── ArduinoOTA/
│   │
│   ├── pid_ctrl/ # PID Controller component
│   │   ├── include/
│   │   │   └── pid_ctrl.h
│   │   ├── src/
│   │   │   └── pid_ctrl.c
│   │   ├── CMakeLists.txt
│   │   └── idf_component.yml
│   ├── trix_PID/ # PID Controller component
│   │   ├── include/
│   │   │   └── trix_PID.h
│   │   ├── pid_ctrl.c
│   │   ├── CMakeLists.txt
│   │   
│   │
│   ├── i2c_bus/         # I2C Bus abstraction component
│   │   ├── include/
│   │   │   └── i2c_bus.h
│   │   ├── src/
│   │   │   ├── i2c_bus.c
│   │   │   ├── i2c_bus_soft.c
│   │   │   └── i2c_bus_v2.c
│   │   ├── CMakeLists.txt
│   │   └── idf_component.yml
│   │
│   ├── mpu6050/         # MPU6050 sensor driver component
│   │   ├── include/
│   │   │   └── mpu6050.h
│   │   ├── src/
│   │   │   └── mpu6050.c
│   │   ├── CMakeLists.txt
│   │   └── idf_component.yml
│   │
│   ├── espressif__pid_ctrl/
│   │	├── include/	
│   │	│	└──pid_ctrl.h
│   │	└── pid_ctrl.c
│   │	        CMakeLists.txt
│   │
│   ├── trix_wifi_connection/
│   │	├── include/	
│   │	│	└──trix_wifi_connection.h
│   │	└── trix_wifi_connection.c
│   │	        CMakeLists.txt
│   └── trix_kalman_filter/
│	├── include/	
│	│	└──trix_kalman_filter.h
│	└── trix_kalman_filter.c
│	        CMakeLists.txt
│   └── uros_helper/
│	├── include/	
│	│	└──.h
│	└── src/
│
│
│
├── main/                # Your primary application code
│   ├── main.cpp         # Main application logic
│   └── CMakeLists.txt   # CMake build configuration for your 'main' component
│
├── CMakeLists.txt       # Main project CMakeLists.txt (for the entire /esp2024 project)
├── Dockerfile           # Dockerfile for building the Docker image
└── docker-compose.yml   # Docker Compose configuration for multi-container setup




sudo docker system prune -a
docker system prune -a --volumes
sudo docker-compose build
docker exec -it robot bash

source /opt/esp-idf/export.sh


idf.py fullclean
idf.py build
idf.py -p /dev/ttyUSB0 flash 
idf.py -p /dev/ttyUSB0 flash monitor
idf.py -p /dev/ttyUSB0 monitor

sudo docker stop $(sudo docker ps -aq) 
sudo docker stop robot
sudo docker rm robot
sudo docker rmi ubuntu:robot # Or whatever your image name is

# Elimina todos los contenedores y volúmenes
docker rm -f $(docker ps -aq) 2>/dev/null
docker volume rm $(docker volume ls -q) 2>/dev/null

# Limpia el sistema Docker
docker system prune -af
sudo docker system prune -a --volumes
sudo docker build -t ubuntu:robot . # Make sure the tag matches what you use in docker-compose.yml
sudo docker-compose up --build -d
docker build --no-cache --progress=plain -t esp-dev .

para leer suscp ros 


source /opt/ros/humble/setup.bash 

 fastdds discovery -i 0



 sudo tcpdump -ni any udp port 9999

source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
ros2 topic list
ros2 topic echo /drone_telemetry_31A2C4




jose@jose-K45VD:~/Descargas/2025master-DRONespui-20250813T180413Z-1-001/2025master-DRONespui$ docker exec -it micro_ros_agent bash -lc '
  source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
  for i in $(seq 0 30); do
    export ROS_DOMAIN_ID=$i
    ros2 daemon stop >/dev/null 2>&1
    T=$(ros2 topic list -t --no-daemon | awk "/std_msgs\\/msg\\/String/{print \$1}" | xargs)
    if [ -n "$T" ]; then
      echo ">>> FOUND Domain=$i  topics: $T"
    fi
  done
'
>>> FOUND Domain=0  topics: /drone_telemetry_31A2C4








