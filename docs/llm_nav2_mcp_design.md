# LLM and Nav2 MCP Design Notes

This note captures how an LLM-facing tool can use a ROS 2 occupancy map,
robot pose, Nav2, and depth data to produce clean navigation outputs such as
room-corner coordinates.

## What `/map` Provides

The `/map` topic is normally a `nav_msgs/msg/OccupancyGrid`. It is not an RViz
image. RViz renders this message visually, but the actual input is a grid of
occupancy values plus metadata that maps grid cells into metric coordinates.

Important fields:

```text
header.frame_id
info.resolution
info.width
info.height
info.origin.position.x
info.origin.position.y
info.origin.orientation
data
```

Typical meaning:

```text
header.frame_id         Coordinate frame, usually "map"
info.resolution         Meters per grid cell, for example 0.05
info.width              Number of cells in grid x direction
info.height             Number of cells in grid y direction
info.origin             Pose of grid cell (0, 0) in the map frame
data                    Flat row-major int8 array of occupancy values
```

Common occupancy values:

```text
-1   unknown
0    free
100  occupied
```

Some systems publish intermediate probabilities from 1 to 99. A practical
consumer should use thresholds instead of assuming only three values.

For a grid cell `(cell_x, cell_y)`, the usual map coordinate conversion is:

```text
map_x = origin_x + (cell_x + 0.5) * resolution
map_y = origin_y + (cell_y + 0.5) * resolution
```

If `info.origin.orientation` has non-zero yaw, the cell coordinate should also
be rotated by that origin pose. Most 2D SLAM maps are axis-aligned with zero yaw,
but code should not silently assume that.

The index into `data` is:

```text
index = cell_y * width + cell_x
value = data[index]
```

The map describes walls, obstacles, free cells, and unknown cells. It does not
by itself tell the current robot position. For robot pose, use one of:

```text
map -> base_link or map -> base_footprint from /tf
/amcl_pose
/odom, if map-frame localization is not available
```

For navigation safety, `/global_costmap/costmap` is also useful because it
usually includes inflation around obstacles and the robot footprint.

## How to Interpret a Room Corner

The map does not label any cell as "corner". A corner is inferred from geometry.

A useful operational definition:

```text
A room corner is where two wall-like occupied boundaries meet at roughly a
right angle, with reachable free space on the inside.
```

The robot should not be sent to the wall intersection itself. The goal should be
offset inward into reachable free space by at least the robot radius plus a
safety margin.

Recommended algorithm:

1. Read the latest `OccupancyGrid` from `/map`.
2. Classify cells into free, occupied, and unknown using thresholds.
3. Inflate occupied cells by `robot_radius + safety_margin`.
4. Use the current robot pose and flood fill to identify the reachable free
   region.
5. Extract boundary cells where reachable free space touches occupied or
   unknown cells.
6. Fit wall segments to boundaries using a contour method, Ramer-Douglas-Peucker
   simplification, or a Hough line transform.
7. Find intersections of long wall segments whose angle difference is close to
   90 degrees.
8. For each intersection, sample inward offsets and choose points that are:
   reachable, free after inflation, not unknown, and accepted by the costmap.
9. Score candidates by angle quality, wall support length, clearance, distance,
   and requested policy such as nearest, farthest, front-left, or all.
10. Return clean goal coordinates in the `map` frame.

Candidate output should look like:

```json
{
  "frame_id": "map",
  "corners": [
    {
      "id": "corner_1",
      "goal": {"x": 1.25, "y": -0.80, "yaw": 2.35},
      "wall_intersection": {"x": 0.80, "y": -1.20},
      "clearance_m": 0.45,
      "distance_from_robot_m": 2.1,
      "score": 0.91,
      "reachable": true,
      "notes": "Goal is offset inside free space; not the physical wall point."
    }
  ]
}
```

## Current ROS-MCP Check

Checked from this workspace on 2026-06-24.

The available ROS-MCP tool surface has generic ROS operations:

```text
connect_to_robot
get_topics / get_topic_type / get_topic_details
subscribe_once / subscribe_for_duration
publish_once / publish_for_durations
get_services / call_service
get_actions / send_action_goal
get_parameters / set_parameter
```

Current runtime result from the connected ROS-MCP endpoint:

```text
get_topics:    No topics found
get_services:  No services found
get_actions:   Action listing not supported by this rosbridge/rosapi version
detect version: ROS version not detected from the current endpoint
```

Interpretation:

The generic ROS-MCP bridge exists, but the currently connected ROS graph is not
exposing topics/services through it. Also, this rosbridge/rosapi version does
not expose action listing via `/rosapi/action_servers`. That does not prove Nav2
cannot be used, but it means an LLM should not rely on action discovery here.
If the action name and type are known, a tool can still attempt to send a
`/navigate_to_pose` goal directly.

This ROS-MCP surface does not currently provide higher-level tools like:

```text
detect_corners(number_of_corners_required)
go_to_coordinate(x, y, yaw)
read_depth_cloud()
validate_goal(x, y)
summarize_map()
```

Those should be added as deterministic robot-side tools.

## Existing Repo Support

This repository already has useful building blocks in
`src/ugv_main/ugv_semantic_navigation`.

Existing capabilities:

```text
semantic_navigation.launch.py starts rosbridge and rosapi for ROS-MCP.
semantic_memory_mcp_server.py exposes semantic-memory MCP tools.
biased_frontier_explorer.py reads /map, reads /odom or TF, detects frontiers,
and sends Nav2 NavigateToPose goals.
semantic_nav_node.py reads /global_costmap/costmap, validates standoff poses,
and sends Nav2 NavigateToPose goals to /navigate_to_pose.
```

Existing semantic-memory MCP tools:

```text
semantic_search
navigation_candidates
semantic_object
semantic_labels
semantic_store_info
```

What is missing for the use case in this note:

```text
No MCP tool currently exposes clean geometric map reasoning such as
detect_corners().

No MCP tool currently exposes a clean generic go_to_coordinate() wrapper.

No MCP tool currently exposes a compact depth/point-cloud summary.
```

## Recommended Architecture

Yes, making an MCP server for clean Nav2/map/depth operations is a good design.
The LLM should not have to inspect raw occupancy arrays, raw TF streams, or raw
point clouds unless debugging. The robot-side code should turn those streams
into stable, small, typed outputs.

Recommended server:

```text
ugv_navigation_mcp_server
```

Preferred implementation:

```text
Python package inside ugv_semantic_navigation or a new ugv_navigation_tools
package.

Use rclpy directly for /map, /global_costmap/costmap, TF, Nav2 actions, and
depth topics.

Run a background rclpy node that caches the latest map, costmap, robot pose, and
depth data.

Expose FastMCP tools that read from that cache and return compact JSON.
```

This is better than doing all reasoning through generic rosbridge calls because
direct `rclpy` gives better control over QoS, TF lookup, action clients, and
large sensor messages.

## Proposed MCP Tools

### `get_robot_pose`

Purpose:

```text
Return the current robot pose in the map frame.
```

Inputs:

```json
{"frame_id": "map", "base_frame": "base_footprint"}
```

Output:

```json
{
  "frame_id": "map",
  "base_frame": "base_footprint",
  "x": 0.0,
  "y": 0.0,
  "yaw": 0.0,
  "source": "tf",
  "age_sec": 0.04
}
```

### `summarize_map`

Purpose:

```text
Return metadata and simple statistics about the latest map.
```

Output:

```json
{
  "frame_id": "map",
  "resolution": 0.05,
  "width": 384,
  "height": 384,
  "origin": {"x": -10.0, "y": -10.0, "yaw": 0.0},
  "free_percent": 42.0,
  "occupied_percent": 18.0,
  "unknown_percent": 40.0
}
```

### `detect_corners`

Purpose:

```text
Detect reachable room-corner goal coordinates from /map.
```

Inputs:

```json
{
  "number_of_corners_required": 4,
  "policy": "nearest",
  "min_clearance_m": 0.45,
  "use_unknown_as_wall": false,
  "frame_id": "map"
}
```

Output:

```json
{
  "frame_id": "map",
  "count": 4,
  "corners": [
    {
      "id": "corner_1",
      "x": 1.25,
      "y": -0.80,
      "yaw": 2.35,
      "clearance_m": 0.48,
      "distance_from_robot_m": 2.1,
      "score": 0.91,
      "reachable": true
    }
  ]
}
```

### `validate_goal`

Purpose:

```text
Check whether a coordinate is safe before navigation.
```

Inputs:

```json
{"x": 1.25, "y": -0.80, "yaw": 2.35, "frame_id": "map"}
```

Output:

```json
{
  "valid": true,
  "reason": "reachable_free_space",
  "clearance_m": 0.48,
  "costmap_value": 0
}
```

Optional improvement:

```text
Call Nav2 ComputePathToPose to verify that the global planner can produce a
path, not only that the cell is locally free.
```

### `go_to_coordinate`

Purpose:

```text
Send a Nav2 goal to a coordinate.
```

Inputs:

```json
{
  "x": 1.25,
  "y": -0.80,
  "yaw": 2.35,
  "frame_id": "map",
  "wait": true,
  "timeout_sec": 60
}
```

Output:

```json
{
  "accepted": true,
  "status": "succeeded",
  "final_pose": {"x": 1.24, "y": -0.82, "yaw": 2.30},
  "message": "Nav2 reached the goal."
}
```

Implementation should use `nav2_msgs/action/NavigateToPose` on
`/navigate_to_pose`.

### `read_depth_cloud`

Purpose:

```text
Return a compact depth or point-cloud summary, not a huge raw point cloud.
```

Inputs:

```json
{
  "topic": "/oak/stereo/points",
  "frame_id": "camera_link",
  "roi": "center",
  "max_points": 2000
}
```

Output:

```json
{
  "frame_id": "camera_link",
  "nearest_obstacle_m": 0.72,
  "center_depth_m": 1.85,
  "valid_points": 1840,
  "obstacle_bearing_deg": -8.0,
  "summary": "Closest obstacle is slightly left of center."
}
```

Depending on the running camera driver, the source may be a depth image such as
`/oak/stereo/image_raw` or `/camera/depth/image_raw`, or a
`sensor_msgs/msg/PointCloud2` topic.

## Suggested Development Plan

1. Add a `navigation_context.py` module that maintains cached `/map`,
   `/global_costmap/costmap`, TF robot pose, and depth/point-cloud summaries.
2. Add deterministic geometry helpers:
   `grid_to_world`, `world_to_grid`, `inflate_obstacles`, `reachable_mask`,
   `extract_boundaries`, `detect_corner_candidates`, and `score_corners`.
3. Add a `ugv_navigation_mcp_server.py` FastMCP executable with:
   `get_robot_pose`, `summarize_map`, `detect_corners`, `validate_goal`,
   `go_to_coordinate`, and `read_depth_cloud`.
4. Reuse existing Nav2 goal logic from `semantic_nav_node.py` instead of
   creating a separate action-client style.
5. Add debug publishers for RViz:
   detected corner markers, selected goal pose, rejected candidates, and
   reachable free-space mask.
6. Add tests using synthetic occupancy grids:
   rectangular room, rotated room, L-shaped room, obstacle near corner, unknown
   frontier, and no reachable corner.

## Practical LLM Workflow

For a user request like:

```text
Go to the nearest corner.
```

The LLM should call tools in this order:

```text
1. get_robot_pose()
2. summarize_map()
3. detect_corners(number_of_corners_required=4, policy="nearest")
4. validate_goal(x, y, yaw)
5. go_to_coordinate(x, y, yaw)
```

For a request like:

```text
Go to a corner and inspect what is in front of you.
```

Use:

```text
1. detect_corners(...)
2. go_to_coordinate(...)
3. read_depth_cloud(...)
4. optionally capture/read camera image
```

## Key Recommendation

Build the MCP layer as a deterministic robot-navigation API, not as a raw ROS
data pipe for the LLM. The LLM should decide intent and policy, such as
"nearest corner" or "farthest corner", while robot-side code should handle map
geometry, safety margins, reachability, TF, costmap checks, and Nav2 action
execution.

## Implementation Added

A separate Python ROS 2 package has been added at:

```text
src/ugv_main/ugv_navigation_mcp
```

It provides the `navigation_mcp_server` executable and these MCP tools:

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

Default topics match this UGV first:

```text
/map
/global_costmap/costmap
/odom
/navigate_to_pose
/oak/stereo/image_raw
```

Run after building and sourcing the workspace:

```bash
ros2 run ugv_navigation_mcp navigation_mcp_server
```

For HTTP-style MCP transport:

```bash
ros2 run ugv_navigation_mcp navigation_mcp_server \
  --transport streamable-http --host 127.0.0.1 --port 8766
```

The pure map geometry lives in `ugv_navigation_mcp/grid_geometry.py`, so corner
detection and goal validation can be tested without a live robot. The ROS/FastMCP
adapter lives in `navigation_context.py` and `navigation_mcp_server.py`.
