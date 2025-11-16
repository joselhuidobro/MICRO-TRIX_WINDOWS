#!/usr/bin/env bash
# Abre gzclient dentro del contenedor, conectando al gzserver ya corriendo.
# Uso: ./gazebo/gazebo-gui.sh [NOMBRE_CONTENEDOR]
set -euo pipefail

CID="${1:-gazebo_sim}"

# Permitir al contenedor usar tu X11
xhost +local:root >/dev/null 2>&1 || true

docker exec -it \
  -e DISPLAY="$DISPLAY" \
  "$CID" \
  bash -lc '
    source /usr/share/gazebo/setup.sh
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    export LIBGL_ALWAYS_SOFTWARE=1
    export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
    export OGRE_RTT_MODE=Copy
    # Rutas de recursos (shaders, modelos, worlds)
    export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:${GAZEBO_RESOURCE_PATH}
    export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH}
    # Conexión al master dentro del MISMO contenedor
    export GAZEBO_MASTER_URI=http://127.0.0.1:11345

    # Espera corta a que el master esté listo
    for i in {1..30}; do
      if gz topic -l >/dev/null 2>&1; then break; fi
      sleep 0.3
    done

    gzclient --verbose
  '
# Cuando termines, si quieres: xhost -local:root

