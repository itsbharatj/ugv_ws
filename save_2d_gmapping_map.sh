#!/usr/bin/env bash
set -e

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$script_dir/src/ugv_main/ugv_nav/maps"
ros2 run nav2_map_server map_saver_cli -f ./map
cd -
