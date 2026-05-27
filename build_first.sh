#!/usr/bin/env bash
set -e

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$script_dir"

source /opt/ros/humble/setup.bash
nav2_overlay="$workspace_dir/.deps/nav2_overlay/opt/ros/humble"
if [ -d "$nav2_overlay" ]; then
  export AMENT_PREFIX_PATH="$nav2_overlay:${AMENT_PREFIX_PATH:-}"
  export CMAKE_PREFIX_PATH="$nav2_overlay:${CMAKE_PREFIX_PATH:-}"
  export LD_LIBRARY_PATH="$nav2_overlay/lib:${LD_LIBRARY_PATH:-}"
  export PYTHONPATH="$nav2_overlay/local/lib/python3.10/site-packages:${PYTHONPATH:-}"
fi
cd "$workspace_dir"

colcon build --packages-select apriltag apriltag_msgs apriltag_ros cartographer costmap_converter_msgs costmap_converter emcl2 explore_lite openslam_gmapping slam_gmapping ldlidar rf2o_laser_odometry robot_pose_publisher teb_msgs teb_local_planner vizanti vizanti_cpp vizanti_demos vizanti_msgs vizanti_server ugv_base_node ugv_interface

if [ -f "$workspace_dir/install/setup.bash" ]; then
  source "$workspace_dir/install/setup.bash"
fi

colcon build --packages-select ugv_bringup ugv_chat_ai ugv_description ugv_gazebo ugv_nav ugv_slam ugv_tools ugv_vision ugv_web_app

if ! grep -q 'source /opt/ros/humble/setup.bash' ~/.bashrc; then
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
fi

if command -v register-python-argcomplete >/dev/null 2>&1; then
  if ! grep -q 'register-python-argcomplete ros2' ~/.bashrc; then
    echo 'eval "$(register-python-argcomplete ros2)"' >> ~/.bashrc
  fi
  if ! grep -q 'register-python-argcomplete colcon' ~/.bashrc; then
    echo 'eval "$(register-python-argcomplete colcon)"' >> ~/.bashrc
  fi
fi

if ! grep -q "source $workspace_dir/install/setup.bash" ~/.bashrc; then
  echo "source $workspace_dir/install/setup.bash" >> ~/.bashrc
fi

source ~/.bashrc
