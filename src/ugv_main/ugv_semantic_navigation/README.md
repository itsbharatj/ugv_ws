# UGV Semantic Navigation

This package adapts the tested `ROS-MCP/demos-ros-mcp-server/10_semantic_navigation`
pipeline to the physical UGV rover. It reuses the rover's existing LiDAR,
odometry, GMapping, Nav2, robot description, motor driver, and OAK-D launch
files.

`ugv_nav/slam_nav.launch.py` builds `/map` and runs Nav2. The frontier explorer
sends Nav2 goals. At each reached frontier, the OAK-D observer rotates the
rover, saves images with map poses, and stores captures and detected object
metadata in JSON and SQLite. Semantic memory can be queried over ROS topics or
rosbridge, and `/semantic_nav/go_to_object` sends an object standoff goal.

```bash
cd ~/ugv_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ugv_semantic_navigation semantic_navigation.launch.py
```

Runtime data is stored under `~/.ros/ugv_semantic_navigation/`.
The semantic launch uses `rover_base_frame:=base_footprint` separately from
the OAK-D driver's camera-frame launch arguments.

Set `OPENAI_API_KEY` before launch to label captured objects and generate
embeddings. Without it, the observer still saves each OAK-D image and its map
pose/metadata to SQLite as an image-only capture. Frontier exploration,
rosbridge/rosapi access, stored-memory retrieval, and navigation remain
available.

Useful launch overrides include `duration:=300`, `capture_angles:=4`,
`capture_radius:=1.0`, `area_captured_min_views:=3`,
`frontier_arrival_tolerance:=0.45`, `explore:=false`, `camera:=false`, and
`rover_bringup:=false`. The frontier arrival tolerance is only a near-arrival
status threshold; semantic capture starts after Nav2 reports the goal reached.
Set `extra_exploration:=true` to let the explorer visit a small number of
under-covered free-space viewpoints after frontiers are exhausted. The default
extra goal count is `extra_exploration_goals:=3`; the mode is disabled unless
explicitly enabled.

Set `start_fresh:=true` to archive the existing runtime directory before the
semantic nodes start. For example, `~/.ros/ugv_semantic_navigation` is renamed
to a timestamped sibling directory, then a clean runtime directory is created
for new images, JSON, and SQLite memory.
Physical motion is interlocked by `/voltage`; the
default `min_voltage:=10.5` stops exploration and rejects capture/navigation
requests on a low battery. Set `min_voltage:=0` only when an external battery
safety system is active.

```bash
ros2 topic pub --once /semantic_capture/request std_msgs/msg/String \
  "{data: '{\"request_id\":\"manual_1\",\"reason\":\"manual\"}'}"
ros2 topic pub --once /semantic_memory/query std_msgs/msg/String \
  "{data: '{\"query\":\"chair\",\"top_k\":5}'}"
ros2 topic pub --once /semantic_nav/go_to_object std_msgs/msg/String \
  "{data: '{\"label\":\"chair\",\"match\":\"best\"}'}"
```

The launch starts rosbridge and rosapi for ROS-MCP. The optional direct
semantic-memory MCP executable requires `fastmcp`:

```bash
python3 -m pip install --user fastmcp
ros2 run ugv_semantic_navigation semantic_memory_mcp_server \
  --vector-store ~/.ros/ugv_semantic_navigation/semantic_memory.sqlite3
```
