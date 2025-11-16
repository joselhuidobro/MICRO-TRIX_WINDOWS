#!/usr/bin/env bash
set -e

# GUI amigable con X11/Wayland y sin audio
export QT_X11_NO_MITSHM=1
export QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-xcb}
export SDL_AUDIODRIVER=${SDL_AUDIODRIVER:-dummy}
export GAZEBO_AUDIO_DEVICE=${GAZEBO_AUDIO_DEVICE:-none}

# ROS 2
source /opt/ros/humble/setup.bash

# Asegura rutas (por si vienen vacías en runtime)
export GAZEBO_PLUGIN_PATH="/opt/ros/humble/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${GAZEBO_PLUGIN_PATH}"
export GAZEBO_MODEL_PATH="/sim/models:/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}"
export GAZEBO_RESOURCE_PATH="/sim/worlds:/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH}"

WORLD="${WORLD:-/usr/share/gazebo-11/worlds/empty.world}"

if [ "${1:-run}" = "run" ]; then
  # Usa el launch oficial: carga gazebo_ros_init/factory/clock automáticamente
  if [ "${GZ_HEADLESS:-0}" = "1" ]; then
    exec ros2 launch gazebo_ros gazebo.launch.py world:="$WORLD" gui:=false
  else
    exec ros2 launch gazebo_ros gazebo.launch.py world:="$WORLD" gui:=true
  fi
fi

# Permite ejecutar otros comandos
exec "$@"

