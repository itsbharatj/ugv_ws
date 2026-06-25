#!/usr/bin/env bash

export UGV_MODEL="${UGV_MODEL:-ugv_rover}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"

if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash
fi

nav2_overlay="/home/bharat/ugv_ws/.deps/nav2_overlay/opt/ros/humble"
if [ -d "${nav2_overlay}" ]; then
  export AMENT_PREFIX_PATH="${nav2_overlay}:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="${nav2_overlay}:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="${nav2_overlay}/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="${nav2_overlay}/local/lib/python3.10/site-packages:${PYTHONPATH:-}"
fi

if [ -f /home/bharat/ugv_ws/install/setup.bash ]; then
  source /home/bharat/ugv_ws/install/setup.bash
fi
