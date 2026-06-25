# UGV Docker GUI Runtime

This container provides two remote GUI paths:

- Browser/noVNC: `http://<host-ip>:6080/vnc.html`, password `bharat`
- SSH/X11: `ssh -p 23 root@<host-ip>`, password `bharat`

The workspace is mounted at `/home/bharat/ugv_ws`, and shell sessions source ROS Humble plus `/home/bharat/ugv_ws/install/setup.bash` automatically.

Start it from the workspace root:

```bash
./docker/start_gui.sh
```

Run a GUI launch inside the noVNC terminal or an SSH/X11 session:

```bash
ros2 launch ugv_description display.launch.py use_rviz:=true
```
