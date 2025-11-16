###############################################################################
# Stage 0 – deps ROS (opcional, útil si generas interfaces)
###############################################################################
FROM ros:humble-ros-base-jammy AS rosdeps
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    git \
 && rosdep init && rosdep update \
 && rm -rf /var/lib/apt/lists/*

###############################################################################
# Stage 1 – builder: ESP-IDF v5.x + micro-ROS listo
###############################################################################
FROM espressif/idf:release-v5.1 AS builder

# ---------- Vars de entorno ----------
ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8 \
    IDF_CCACHE_ENABLE=1 \
    IDF_PATH=/opt/esp/idf

# ---------- Repos de ROS2 ----------
RUN mkdir -p /usr/share/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
      -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    bash -lc 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/lsb-release && echo $DISTRIB_CODENAME) main" > /etc/apt/sources.list.d/ros2.list'

# ---------- Paquetes mínimos ----------
RUN apt-get update && apt-get install -y --no-install-recommends \
      git curl make build-essential \
      python3-pip python3-venv python3-setuptools python3-serial \
      python3-vcstool python3-colcon-common-extensions \
      python3-rosdep \
      libmagic-dev \
  && rosdep init && rosdep update \
  && rm -rf /var/lib/apt/lists/*

# ---------- Venv Python para herramientas rosidl/colcon ----------
RUN python3 -m venv /opt/ros_venv && \
    /opt/ros_venv/bin/pip install --no-cache-dir \
      "empy<4" catkin_pkg lark-parser pyyaml vcstool colcon-common-extensions
ENV PATH="/opt/ros_venv/bin:${PATH}"

# ---------- Fix de empy en el venv de ESP-IDF ----------
RUN . ${IDF_PATH}/export.sh && \
    python -m pip uninstall -y empy || true && \
    python -m pip install --no-cache-dir "empy<4" catkin_pkg lark-parser pyyaml vcstool colcon-common-extensions

# ---------- micro-ROS fuera del proyecto ----------
ARG MICROROS_TAG=humble
RUN rm -rf /opt/extra_components && mkdir -p /opt/extra_components && \
    git clone -b ${MICROROS_TAG} --depth 1 \
      https://github.com/micro-ROS/micro_ros_espidf_component.git \
      /opt/extra_components/micro_ros_espidf_component



# ---------- Proyecto del usuario ----------
WORKDIR /esp2024
COPY . /esp2024
COPY sdkconfig.defaults /esp2024/sdkconfig.defaults

# Deps Python dentro del entorno IDF (extra)
RUN . ${IDF_PATH}/export.sh && \
    pip install --no-cache-dir "empy<4" catkin_pkg lark-parser pyyaml vcstool colcon-common-extensions



# ---------- Reconfigura el proyecto ----------
#RUN . ${IDF_PATH}/export.sh && idf.py reconfigure

# ---------- Permisos scripts locales ----------
RUN find /esp2024 -type f -name "*.sh" -exec chmod +x {} + || true

# ---------- Usuario ----------
RUN echo "source ${IDF_PATH}/export.sh" >> ~/.bashrc

ENTRYPOINT ["/bin/bash"]

