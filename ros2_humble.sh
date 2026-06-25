#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
container_name="ugv_jetson_ros_humble"

"${script_dir}/docker/start_gui.sh"

echo "Entering ${container_name}..."
docker exec -it "${container_name}" bash
