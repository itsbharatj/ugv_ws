#!/usr/bin/env bash
set -e

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$script_dir/src/ugv_main/ugv_nav/maps"
ros2 run nav2_map_server map_saver_cli -f ./map
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$script_dir/src/ugv_main/ugv_nav/maps/map.pbstream'}"
cd -
