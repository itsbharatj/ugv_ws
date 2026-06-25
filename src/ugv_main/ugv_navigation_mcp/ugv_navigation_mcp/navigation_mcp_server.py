#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from typing import Any

from ugv_navigation_mcp.navigation_context import TopicConfig, start_ros_runtime


TOOL_NAMES = [
    'server_status',
    'list_navigation_topics',
    'configure_topics',
    'get_robot_pose',
    'summarize_map',
    'detect_corners',
    'validate_goal',
    'go_to_coordinate',
    'read_depth_cloud',
]


def build_server(context: Any):
    try:
        from fastmcp import FastMCP
    except ImportError as exc:
        raise RuntimeError(
            'fastmcp is required to run navigation_mcp_server. '
            'Install it with: python3 -m pip install --user fastmcp'
        ) from exc

    mcp = FastMCP('ugv-navigation')

    @mcp.tool
    def server_status() -> dict[str, Any]:
        """Return configured topics and whether map, costmap, pose, and depth data have arrived."""
        return context.server_status()

    @mcp.tool
    def list_navigation_topics() -> dict[str, Any]:
        """List ROS topic suggestions for map, costmap, odometry, depth image, and point cloud inputs."""
        return context.list_navigation_topics()

    @mcp.tool
    def configure_topics(
        map_topic: str | None = None,
        costmap_topic: str | None = None,
        odom_topic: str | None = None,
        depth_image_topic: str | None = None,
        pointcloud_topic: str | None = None,
        navigate_action: str | None = None,
        map_frame: str | None = None,
        base_frame: str | None = None,
        camera_hfov_deg: float | None = None,
    ) -> dict[str, Any]:
        """Update topic/frame/action configuration and resubscribe where needed."""
        return context.configure_topics(
            map_topic=map_topic,
            costmap_topic=costmap_topic,
            odom_topic=odom_topic,
            depth_image_topic=depth_image_topic,
            pointcloud_topic=pointcloud_topic,
            navigate_action=navigate_action,
            map_frame=map_frame,
            base_frame=base_frame,
            camera_hfov_deg=camera_hfov_deg,
        )

    @mcp.tool
    def get_robot_pose(
        frame_id: str | None = None,
        base_frame: str | None = None,
        timeout_sec: float = 0.08,
    ) -> dict[str, Any]:
        """Return the current robot pose, preferring TF and falling back to odometry."""
        return context.get_robot_pose(
            frame_id=frame_id,
            base_frame=base_frame,
            timeout_sec=timeout_sec,
        )

    @mcp.tool
    def summarize_map(wait_timeout_sec: float = 2.0) -> dict[str, Any]:
        """Return compact metadata and occupancy statistics for the latest map."""
        return context.summarize_map(wait_timeout_sec=wait_timeout_sec)

    @mcp.tool
    def detect_corners(
        number_of_corners_required: int = 4,
        policy: str = 'nearest',
        min_clearance_m: float = 0.45,
        max_corner_offset_m: float = 1.2,
        use_unknown_as_wall: bool = False,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        """Return reachable room-corner goal coordinates in the map frame."""
        return context.detect_corners(
            number_of_corners_required=number_of_corners_required,
            policy=policy,
            min_clearance_m=min_clearance_m,
            max_corner_offset_m=max_corner_offset_m,
            use_unknown_as_wall=use_unknown_as_wall,
            wait_timeout_sec=wait_timeout_sec,
        )

    @mcp.tool
    def validate_goal(
        x: float,
        y: float,
        min_clearance_m: float = 0.45,
        allow_unknown: bool = False,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        """Check whether a map-frame coordinate is free, clear, and reachable."""
        return context.validate_goal(
            x=x,
            y=y,
            min_clearance_m=min_clearance_m,
            allow_unknown=allow_unknown,
            wait_timeout_sec=wait_timeout_sec,
        )

    @mcp.tool
    def go_to_coordinate(
        x: float,
        y: float,
        yaw: float = 0.0,
        frame_id: str | None = None,
        wait: bool = True,
        timeout_sec: float = 60.0,
        validate: bool = True,
        min_clearance_m: float = 0.45,
    ) -> dict[str, Any]:
        """Send a Nav2 NavigateToPose goal to a coordinate."""
        return context.go_to_coordinate(
            x=x,
            y=y,
            yaw=yaw,
            frame_id=frame_id,
            wait=wait,
            timeout_sec=timeout_sec,
            validate=validate,
            min_clearance_m=min_clearance_m,
        )

    @mcp.tool
    def read_depth_cloud(
        source: str = 'auto',
        roi: str = 'center',
        max_points: int = 2000,
    ) -> dict[str, Any]:
        """Return a compact depth-image or point-cloud summary."""
        return context.read_depth_cloud(
            source=source,
            roi=roi,
            max_points=max_points,
        )

    return mcp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='FastMCP server for map geometry and Nav2 actions')
    parser.add_argument('--transport', default='stdio', choices=['stdio', 'sse', 'streamable-http'])
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=8766)
    parser.add_argument('--map-topic', default='/map')
    parser.add_argument('--costmap-topic', default='/global_costmap/costmap')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--depth-image-topic', default='/oak/stereo/image_raw')
    parser.add_argument('--pointcloud-topic', default='')
    parser.add_argument('--navigate-action', default='/navigate_to_pose')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--base-frame', default='base_footprint')
    parser.add_argument('--camera-hfov-deg', type=float, default=72.0)
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument(
        '--dump-tools',
        action='store_true',
        help='Print a JSON description of tools and defaults without starting ROS or MCP',
    )
    return parser.parse_args()


def default_config_from_args(args: argparse.Namespace) -> TopicConfig:
    return TopicConfig(
        map_topic=args.map_topic,
        costmap_topic=args.costmap_topic,
        odom_topic=args.odom_topic,
        depth_image_topic=args.depth_image_topic,
        pointcloud_topic=args.pointcloud_topic,
        navigate_action=args.navigate_action,
        map_frame=args.map_frame,
        base_frame=args.base_frame,
        use_sim_time=args.use_sim_time,
        camera_hfov_deg=args.camera_hfov_deg,
    )


def main() -> None:
    args = parse_args()
    config = default_config_from_args(args)
    if args.dump_tools:
        print(json.dumps({
            'server': 'ugv-navigation',
            'tools': TOOL_NAMES,
            'default_config': config.__dict__,
        }, indent=2, sort_keys=True))
        return

    runtime = start_ros_runtime(config)
    try:
        mcp = build_server(runtime.context)
        if args.transport == 'stdio':
            mcp.run(transport='stdio')
        else:
            mcp.run(transport=args.transport, host=args.host, port=args.port)
    finally:
        runtime.shutdown()


if __name__ == '__main__':
    main()
