# syntax=docker/dockerfile:1
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive TZ=Etc/UTC

# Gazebo Classic + gazebo_ros + utilidades
RUN apt-get update && apt-get install -y --no-install-recommends \
      gazebo \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-xacro \
      mesa-utils libgl1-mesa-dri libglu1-mesa \
  && rm -rf /var/lib/apt/lists/*

# 🔧 Ajustes por defecto
# (Corrige el typo: /usr/share/, no /usr/shar)
ENV GZ_HEADLESS=0 \
    WORLD=/usr/share/gazebo-11/worlds/empty.world \
    # Añade rutas del sistema + tus rutas (no las pises)
    GAZEBO_MODEL_PATH=/sim/models:/usr/share/gazebo-11/models \
    GAZEBO_RESOURCE_PATH=/sim/worlds:/usr/share/gazebo-11 \
    # 👇 Clave para que Gazebo encuentre los plugins de ROS 2
    GAZEBO_PLUGIN_PATH=/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins

# Autocargar ROS 2 en shells
RUN echo 'source /opt/ros/humble/setup.bash' >> /etc/bash.bashrc \
 && printf '%s\n' 'source /opt/ros/humble/setup.bash' > /etc/profile.d/ros2.sh


# Entrypoint que lanza Gazebo vía ros2 launch (carga plugins ROS automáticamente)
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /sim

ENTRYPOINT ["/entrypoint.sh"]
CMD ["run"]

