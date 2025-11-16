#!/usr/bin/env bash
xhost +SI:localuser:root
docker exec -it gazebo_sim bash -lc '
  source /usr/share/gazebo/setup.sh
  export DISPLAY='"$DISPLAY"'
  export QT_QPA_PLATFORM=xcb
  export QT_X11_NO_MITSHM=1
  export LIBGL_ALWAYS_SOFTWARE=1
  export MESA_LOADER_DRIVER_OVERRIDE=llvmpipe
  export OGRE_RTT_MODE=Copy
  export GAZEBO_MASTER_URI=http://127.0.0.1:11345
  gzclient --verbose
'

