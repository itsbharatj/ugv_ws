#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd "${script_dir}/.." && pwd)"

cd "${workspace_dir}"

docker compose -f docker-compose.gui.yml up -d --build

host_ip="$(ip -4 route get 1.1.1.1 2>/dev/null | awk '{for (i=1; i<=NF; i++) if ($i=="src") {print $(i+1); exit}}')"
host_ip="${host_ip:-$(hostname -I | awk '{print $1}')}"

echo
echo "Docker GUI container is running."
echo "  Browser: http://${host_ip}:6080/vnc.html"
echo "  VNC password: bharat"
echo "  SSH: ssh -p 23 root@${host_ip}"
echo "  SSH password: bharat"
echo
echo "To open a shell inside the container:"
echo "  docker exec -it ugv_jetson_ros_humble bash"
