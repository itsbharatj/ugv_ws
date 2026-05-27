#!/usr/bin/env bash
set -e

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

cd "$script_dir/src/ugv_else/apriltag_ros/apriltag"
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target install
cd "$script_dir"
