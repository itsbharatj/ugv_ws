import unittest
import time
from types import SimpleNamespace

from ugv_semantic_navigation.semantic_vector_store import (
    SemanticVectorStore,
    estimate_object_position,
)
from ugv_semantic_navigation.biased_frontier_explorer import (
    BiasedFrontierExplorer,
    nearest_point_to_centroid,
)
from ugv_semantic_navigation.semantic_camera_observer import OpenAISemanticObserver
from ugv_semantic_navigation.semantic_nav_node import SemanticNavNode


class SemanticVectorStoreTest(unittest.TestCase):
    def test_estimate_object_position_from_centered_bbox(self):
        bearing, x, y = estimate_object_position(
            1.0,
            2.0,
            0.0,
            {'x_min': 0.4, 'x_max': 0.6},
            1.5,
        )

        self.assertEqual(bearing, 0.0)
        self.assertEqual(x, 2.5)
        self.assertEqual(y, 2.0)

    def test_capture_is_searchable_and_fused(self):
        with self.subTest('temporary vector store'):
            import tempfile
            from pathlib import Path

            with tempfile.TemporaryDirectory() as temp_dir:
                store = SemanticVectorStore(Path(temp_dir) / 'memory.sqlite3')
                capture = {
                    'id': 'cap_1',
                    'request_id': 'test_1',
                    'timestamp': '2026-06-11T00:00:00Z',
                    'source': 'openai_vla',
                    'pose': {
                        'frame_id': 'map',
                        'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                        'yaw': 0.0,
                    },
                    'summary': 'A chair is in front of the rover.',
                }
                objects = [{
                    'id': 'obs_1',
                    'label': 'chair',
                    'aliases': ['seat'],
                    'description': 'wooden chair near wall',
                    'confidence': 0.95,
                    'visibility': 'clear',
                    'visible_fraction': 1.0,
                    'bbox': {'x_min': 0.4, 'x_max': 0.6},
                    'distance_m': 1.0,
                    'distance_source': 'lidar_projected',
                    'source': 'openai_vla',
                }]

                store.add_capture(capture, objects, embed=False)

                results = store.search('seat near wall', top_k=3)
                fused = store.get_fused_objects('chair')
                self.assertEqual(results[0]['id'], 'obs_1')
                self.assertEqual(fused[0]['object_x'], 1.0)
                self.assertEqual(fused[0]['object_y'], 0.0)

    def test_area_view_bucket_count_tracks_distinct_yaws(self):
        import tempfile
        from pathlib import Path

        with tempfile.TemporaryDirectory() as temp_dir:
            store = SemanticVectorStore(Path(temp_dir) / 'memory.sqlite3')
            for index, yaw in enumerate((0.0, 2.2, 4.4), start=1):
                store.add_capture({
                    'id': f'cap_{index}',
                    'request_id': 'coverage_test',
                    'timestamp': '2026-06-11T00:00:00Z',
                    'source': 'image_only',
                    'pose': {
                        'frame_id': 'map',
                        'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                        'yaw': yaw,
                    },
                    'yaw': yaw,
                    'summary': 'coverage test',
                }, [], embed=False)

            self.assertEqual(store.area_view_bucket_count(0.0, 0.0, 1.0, 3), 3)
            self.assertTrue(store.area_has_min_views(0.0, 0.0, 1.0, 3))

    def test_frontier_goal_is_an_actual_free_cluster_cell(self):
        concave_cluster = [(0.0, 0.0), (0.0, 2.0), (2.0, 0.0)]

        goal = nearest_point_to_centroid(concave_cluster)

        self.assertIn(goal, concave_cluster)

    def test_extra_exploration_candidates_skip_covered_cells(self):
        import numpy as np

        class StoreStub:
            def area_view_bucket_count(self, x, y, radius, buckets):
                return buckets if x < 2.0 and y < 2.0 else 0

        explorer = object.__new__(BiasedFrontierExplorer)
        explorer.map_data = np.zeros((8, 8), dtype=np.int8)
        explorer.map_info = SimpleNamespace(
            resolution=1.0,
            origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
        )
        explorer.robot_x = 0.0
        explorer.robot_y = 0.0
        explorer.min_goal_distance = 0.0
        explorer.wall_clearance = 0.1
        explorer.extra_exploration_goal_spacing = 2.0
        explorer.extra_exploration_radius = 1.0
        explorer.extra_exploration_min_views = 2
        explorer.extra_exploration_targets_visited = []
        explorer.extra_exploration_blacklisted = []
        explorer.targets_visited = []
        explorer.semantic_vector_store = StoreStub()

        candidates = explorer.find_extra_exploration_candidates()

        self.assertTrue(candidates)
        self.assertFalse(any(candidate.x < 2.0 and candidate.y < 2.0 for candidate in candidates))
        self.assertTrue(all(candidate.missing_views == 2 for candidate in candidates))

    def test_frontier_near_arrival_does_not_cancel_nav2_goal(self):
        class GoalHandle:
            def __init__(self):
                self.cancelled = False

            def cancel_goal_async(self):
                self.cancelled = True

        goal_handle = GoalHandle()
        statuses = []
        explorer = object.__new__(BiasedFrontierExplorer)
        explorer.finished = False
        explorer.exploration_paused = False
        explorer.min_voltage = 0.0
        explorer.latest_voltage = None
        explorer.duration = 0.0
        explorer.start_time = time.monotonic()
        explorer.goal_pending = False
        explorer.current_goal_handle = goal_handle
        explorer.current_target = (1.0, 1.0)
        explorer.robot_x = 0.7
        explorer.robot_y = 1.0
        explorer.arrival_tolerance = 0.45
        explorer.within_arrival_tolerance_reported = False
        explorer.goal_started_at = time.monotonic()
        explorer.goal_timeout = 90.0
        explorer.map_data = object()
        explorer.map_info = object()
        explorer.expire_bias_if_needed = lambda: None
        explorer.update_robot_pose_from_tf = lambda: True
        explorer.target_dict = lambda target: {'x': target[0], 'y': target[1]} if target else None
        explorer.publish_status = lambda state, **extra: statuses.append((state, extra))

        explorer.tick()

        self.assertFalse(goal_handle.cancelled)
        self.assertEqual(statuses[0][0], 'goal_within_arrival_tolerance')
        self.assertTrue(explorer.within_arrival_tolerance_reported)

    def test_all_semantic_motion_nodes_share_low_voltage_interlock(self):
        low_battery = SimpleNamespace(min_voltage=10.5, latest_voltage=10.0)
        charged_battery = SimpleNamespace(min_voltage=10.5, latest_voltage=12.0)

        for node_type in (BiasedFrontierExplorer, OpenAISemanticObserver, SemanticNavNode):
            with self.subTest(node=node_type.__name__):
                self.assertTrue(node_type.voltage_is_low(low_battery))
                self.assertFalse(node_type.voltage_is_low(charged_battery))
