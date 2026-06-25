import math

import numpy as np

from ugv_navigation_mcp.grid_geometry import (
    GridInfo,
    GridSnapshot,
    detect_corners,
    grid_to_world,
    map_summary,
    validate_goal,
    world_to_grid,
)


def rectangular_room(width=60, height=40, resolution=0.1):
    data = np.zeros((height, width), dtype=np.int16)
    data[0, :] = 100
    data[-1, :] = 100
    data[:, 0] = 100
    data[:, -1] = 100
    info = GridInfo(
        resolution=resolution,
        width=width,
        height=height,
        origin_x=0.0,
        origin_y=0.0,
        origin_yaw=0.0,
    )
    return GridSnapshot(data=data, info=info, frame_id='map')


def test_grid_world_round_trip():
    info = GridInfo(
        resolution=0.05,
        width=100,
        height=100,
        origin_x=-2.0,
        origin_y=1.0,
    )
    x, y = grid_to_world(info, 20, 30)
    assert world_to_grid(info, x, y) == (20, 30)


def test_map_summary_counts_occupancy():
    snapshot = rectangular_room(width=10, height=8)
    summary = map_summary(snapshot)
    assert summary['width'] == 10
    assert summary['height'] == 8
    assert summary['occupied_percent'] > 0.0
    assert summary['free_percent'] > summary['occupied_percent']


def test_validate_goal_accepts_clear_free_space_and_rejects_wall():
    snapshot = rectangular_room()
    accepted = validate_goal(
        snapshot=snapshot,
        x=3.0,
        y=2.0,
        min_clearance_m=0.25,
        robot_xy=(3.0, 2.0),
    )
    rejected = validate_goal(
        snapshot=snapshot,
        x=0.05,
        y=0.05,
        min_clearance_m=0.25,
        robot_xy=(3.0, 2.0),
    )
    assert accepted['valid'] is True
    assert rejected['valid'] is False
    assert rejected['reason'] in {'occupied_cell', 'insufficient_clearance'}


def test_detect_corners_returns_four_reachable_goals_in_rectangular_room():
    snapshot = rectangular_room()
    result = detect_corners(
        snapshot=snapshot,
        number_of_corners_required=4,
        robot_xy=(3.0, 2.0),
        min_clearance_m=0.3,
        max_corner_offset_m=0.9,
        policy='nearest',
    )
    assert result['count'] == 4
    assert result['debug']['line_count'] >= 4
    for corner in result['corners']:
        cell_x, cell_y = world_to_grid(snapshot.info, corner['x'], corner['y'])
        assert snapshot.data[cell_y, cell_x] == 0
        assert corner['clearance_m'] >= 0.3
        assert -math.pi <= corner['yaw'] <= math.pi
