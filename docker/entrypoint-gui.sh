#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"
export UGV_MODEL="${UGV_MODEL:-ugv_rover}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export QT_X11_NO_MITSHM="${QT_X11_NO_MITSHM:-1}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"

ROOT_PASSWORD="${ROOT_PASSWORD:-bharat}"
VNC_PASSWORD="${VNC_PASSWORD:-bharat}"
VNC_GEOMETRY="${VNC_GEOMETRY:-1280x800x24}"

mkdir -p /run/sshd "${XDG_RUNTIME_DIR}"
chmod 700 "${XDG_RUNTIME_DIR}"
echo "root:${ROOT_PASSWORD}" | chpasswd

if ! pgrep -x Xvfb >/dev/null 2>&1; then
  Xvfb "${DISPLAY}" -screen 0 "${VNC_GEOMETRY}" -nolisten tcp &
fi

sleep 1

if ! pgrep -x fluxbox >/dev/null 2>&1; then
  fluxbox >/tmp/fluxbox.log 2>&1 &
fi

if ! pgrep -x x11vnc >/dev/null 2>&1; then
  x11vnc -display "${DISPLAY}" -forever -shared -rfbport 5901 -passwd "${VNC_PASSWORD}" -bg -o /tmp/x11vnc.log
fi

if ! pgrep -f "websockify.*6080" >/dev/null 2>&1; then
  websockify --web=/usr/share/novnc/ 6080 localhost:5901 >/tmp/novnc.log 2>&1 &
fi

service ssh start >/dev/null

if command -v xterm >/dev/null 2>&1 && ! pgrep -f "xterm.*ugv_ws" >/dev/null 2>&1; then
  xterm -title "ugv_ws" -geometry 120x32+30+30 -e bash -lc "cd /home/bharat/ugv_ws; source /etc/profile.d/ugv_ros_env.sh; exec bash" >/tmp/xterm.log 2>&1 &
fi

echo "UGV Docker GUI is running."
echo "  noVNC: http://<host-ip>:6080/vnc.html"
echo "  VNC password: ${VNC_PASSWORD}"
echo "  SSH: root@<host-ip> -p 23"
echo "  SSH password: ${ROOT_PASSWORD}"

exec "$@"
