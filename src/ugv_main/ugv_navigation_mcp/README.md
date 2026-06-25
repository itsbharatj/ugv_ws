# UGV Navigation MCP

This package exposes deterministic navigation tools for an LLM/MCP client. It
turns raw ROS 2 map, pose, Nav2, and depth data into compact JSON outputs.

Default topics are set for this UGV:

```text
/map
/global_costmap/costmap
/odom
/navigate_to_pose
/oak/stereo/image_raw
```

The server is configurable with CLI arguments or at runtime through the
`configure_topics` MCP tool, so the same package can be adapted to other robots.

## Tools

```text
server_status
list_navigation_topics
configure_topics
get_robot_pose
summarize_map
detect_corners
validate_goal
go_to_coordinate
read_depth_cloud
```

## Run

Install `fastmcp` in the Python environment used by ROS:

```bash
python3 -m pip install --user fastmcp
```

After building and sourcing the workspace:

```bash
ros2 run ugv_navigation_mcp navigation_mcp_server
```

For a network MCP transport:

```bash
ros2 run ugv_navigation_mcp navigation_mcp_server \
  --transport streamable-http --host 127.0.0.1 --port 8766
```

Useful overrides:

```bash
ros2 run ugv_navigation_mcp navigation_mcp_server \
  --map-topic /map \
  --costmap-topic /global_costmap/costmap \
  --odom-topic /odom \
  --base-frame base_footprint \
  --depth-image-topic /oak/stereo/image_raw
```

## Design

The LLM should choose intent and policy, for example "nearest corner". This
server handles geometry, clearance, reachability, costmap checks, TF lookup,
Nav2 action execution, and compact depth summaries.
