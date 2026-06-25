#!/usr/bin/env python3
"""Nav2-backed biased frontier explorer for Example 10."""

import argparse
import json
import math
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from ugv_semantic_navigation.semantic_vector_store import SemanticVectorStore


FREE = 0
UNKNOWN = -1
OCCUPIED_THRESHOLD = 50


@dataclass
class FrontierCandidate:
    x: float
    y: float
    size_cells: int
    distance: float
    score: float
    bias_score: float
    revisit_penalty: float
    blacklist_penalty: float


@dataclass
class CoverageCandidate:
    x: float
    y: float
    distance: float
    score: float
    view_buckets: int
    missing_views: int
    revisit_penalty: float
    blacklist_penalty: float


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def yaw_from_quaternion(q: Any) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def parse_bool(value: Any) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def nearest_point_to_centroid(cluster: list[tuple[float, float]]) -> tuple[float, float]:
    """Return a real frontier cell near the centroid, never an averaged unknown cell."""
    center_x = sum(point[0] for point in cluster) / len(cluster)
    center_y = sum(point[1] for point in cluster) / len(cluster)
    return min(cluster, key=lambda point: math.hypot(point[0] - center_x, point[1] - center_y))


class BiasedFrontierExplorer(Node):
    """Detect map frontiers, score them with optional JSON bias, and use Nav2."""

    def __init__(self, args: argparse.Namespace):
        super().__init__(
            'biased_frontier_explorer',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time)],
        )

        self.duration = args.duration
        self.min_frontier_size = args.min_frontier_size
        self.goal_timeout = args.goal_timeout
        self.arrival_tolerance = args.arrival_tolerance
        self.min_goal_distance = args.min_goal_distance
        self.replan_period = args.replan_period
        self.wall_clearance = args.wall_clearance
        self.semantic_capture_enabled = args.semantic_capture
        self.semantic_capture_timeout = args.semantic_capture_timeout
        self.base_frame = args.base_frame
        self.min_voltage = max(0.0, float(args.min_voltage))
        self.latest_voltage: float | None = None
        self.extra_exploration_enabled = args.extra_exploration
        self.extra_exploration_goal_limit = max(0, int(args.extra_exploration_goals))
        self.extra_exploration_goal_spacing = max(0.25, float(args.extra_exploration_goal_spacing))
        self.extra_exploration_radius = max(0.2, float(args.extra_exploration_radius))
        self.extra_exploration_min_views = max(1, int(args.extra_exploration_min_views))
        self.extra_exploration_max_attempts = max(
            self.extra_exploration_goal_limit,
            self.extra_exploration_goal_limit * 3,
        )
        self.semantic_vector_store_path = args.semantic_vector_store
        self.semantic_vector_store = (
            SemanticVectorStore(args.semantic_vector_store)
            if self.extra_exploration_enabled and self.extra_exploration_goal_limit > 0
            else None
        )

        self.map_data: np.ndarray | None = None
        self.map_info = None
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.has_odom = False
        self.pose_source = 'unavailable'

        self.bias: dict[str, Any] | None = None
        self.current_target: tuple[float, float] | None = None
        self.current_goal_handle = None
        self.current_goal_kind: str | None = None
        self.goal_pending = False
        self.goal_started_at: float | None = None
        self.goal_sequence = 0
        self.active_goal_id: int | None = None
        self.within_arrival_tolerance_reported = False
        self.goals_succeeded = 0
        self.targets_visited: list[tuple[float, float]] = []
        self.blacklisted: list[tuple[float, float]] = []
        self.extra_exploration_targets_visited: list[tuple[float, float]] = []
        self.extra_exploration_blacklisted: list[tuple[float, float]] = []
        self.extra_exploration_goal_attempts = 0
        self.extra_exploration_goals_completed = 0
        self.extra_exploration_started_reported = False
        self.extra_exploration_finished_reported = False
        self.last_goal_sent_at = 0.0
        self.start_time = time.monotonic()
        self.no_frontier_count = 0
        self.last_candidate_count = 0
        self.capture_pending_request_id: str | None = None
        self.capture_pending_started_at: float | None = None
        self.finished = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/semantic_explorer/status', 10)
        self.capture_request_pub = self.create_publisher(String, '/semantic_capture/request', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)

        self.odom_sub = self.create_subscription(Odometry, args.odom_topic, self.odom_callback, 10)
        self.bias_sub = self.create_subscription(
            String, '/exploration_bias', self.bias_callback, 10)
        self.capture_status_sub = self.create_subscription(
            String, '/semantic_capture/status', self.capture_status_callback, 10)
        self.voltage_sub = self.create_subscription(
            Float32, args.voltage_topic, self.voltage_callback, 10)
        self.pause_sub = self.create_subscription(
            String, '/semantic_explorer/pause', self._pause_callback, 10)
        self.resume_sub = self.create_subscription(
            String, '/semantic_explorer/resume', self._resume_callback, 10)
        self.exploration_paused = False

        map_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)

        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(
            'Biased frontier explorer ready. Waiting for /map, /odom, and Nav2.')

    def odom_callback(self, msg: Odometry) -> None:
        if self.pose_source == 'tf_map':
            self.has_odom = True
            return
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = yaw_from_quaternion(msg.pose.pose.orientation)
        self.has_odom = True
        self.pose_source = 'odom_fallback'

    def map_callback(self, msg: OccupancyGrid) -> None:
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width))
        self.map_info = msg.info

    def voltage_callback(self, msg: Float32) -> None:
        self.latest_voltage = float(msg.data)

    def bias_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.publish_status('bias_error', error=f'invalid JSON: {exc}')
            return

        if payload.get('type') == 'clear':
            self.bias = None
            self.publish_status('bias_cleared')
            return

        bias_type = payload.get('type')
        weight = float(payload.get('weight', 1.0))
        ttl_sec = max(0.0, float(payload.get('ttl_sec', 60.0)))
        expires_at = time.monotonic() + ttl_sec

        if bias_type == 'direction':
            if 'heading_deg' not in payload:
                self.publish_status('bias_error', error='direction bias needs heading_deg')
                return
            self.bias = {
                'type': 'direction',
                'heading_deg': float(payload['heading_deg']),
                'weight': weight,
                'expires_at': expires_at,
                'ttl_sec': ttl_sec,
            }
        elif bias_type == 'point':
            if 'x' not in payload or 'y' not in payload:
                self.publish_status('bias_error', error='point bias needs x and y')
                return
            self.bias = {
                'type': 'point',
                'x': float(payload['x']),
                'y': float(payload['y']),
                'weight': weight,
                'expires_at': expires_at,
                'ttl_sec': ttl_sec,
            }
        else:
            self.publish_status('bias_error', error=f'unsupported bias type: {bias_type}')
            return

        self.publish_status('bias_set', bias=self.public_bias())

    def update_robot_pose_from_tf(self) -> bool:
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return self.has_odom

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        self.robot_x = translation.x
        self.robot_y = translation.y
        self.robot_yaw = yaw_from_quaternion(rotation)
        self.has_odom = True
        self.pose_source = 'tf_map'
        return True

    def _pause_callback(self, msg: String) -> None:
        self.exploration_paused = True
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            self.clear_current_goal()
        self.publish_status('exploration_paused', reason=msg.data)

    def _resume_callback(self, msg: String) -> None:
        self.exploration_paused = False
        self.publish_status('exploration_resumed', reason=msg.data)

    def tick(self) -> None:
        if self.finished:
            return
        if self.exploration_paused:
            return
        if self.min_voltage > 0.0 and self.latest_voltage is None:
            self.publish_status('waiting_for_voltage', min_voltage=self.min_voltage)
            return
        if self.voltage_is_low():
            self.publish_status(
                'low_voltage_stop',
                voltage=round(float(self.latest_voltage), 2),
                min_voltage=self.min_voltage,
            )
            self.stop_exploration()
            return
        self.expire_bias_if_needed()

        if self.duration > 0 and time.monotonic() - self.start_time > self.duration:
            self.publish_status('duration_reached')
            self.stop_exploration()
            self.timer.cancel()
            return

        if not self.update_robot_pose_from_tf():
            self.publish_status('waiting_for_pose')
            return
        if self.map_data is None or self.map_info is None:
            self.publish_status('waiting_for_map')
            return

        if self.goal_pending:
            return

        if self.current_goal_handle is not None:
            if self.current_target is not None:
                distance_to_target = math.hypot(
                    self.current_target[0] - self.robot_x,
                    self.current_target[1] - self.robot_y,
                )
                if (
                    distance_to_target <= self.arrival_tolerance
                    and not self.within_arrival_tolerance_reported
                ):
                    self.within_arrival_tolerance_reported = True
                    self.publish_status(
                        'goal_within_arrival_tolerance',
                        target=self.target_dict(self.current_target),
                        distance_to_target=round(distance_to_target, 3),
                        arrival_tolerance=round(float(self.arrival_tolerance), 3),
                    )
            if self.goal_started_at and time.monotonic() - self.goal_started_at > self.goal_timeout:
                self.publish_status('goal_timeout', target=self.target_dict(self.current_target))
                self.blacklist_current_target(self.current_goal_kind)
                self.current_goal_handle.cancel_goal_async()
                self.clear_current_goal()
            return

        if self.capture_pending_request_id is not None:
            if (
                self.capture_pending_started_at is not None
                and time.monotonic() - self.capture_pending_started_at > self.semantic_capture_timeout
            ):
                request_id = self.capture_pending_request_id
                self.capture_pending_request_id = None
                self.capture_pending_started_at = None
                self.publish_status('semantic_capture_timeout', request_id=request_id)
            else:
                return

        if time.monotonic() - self.last_goal_sent_at < self.replan_period:
            return

        candidates = self.find_frontier_candidates()
        self.last_candidate_count = len(candidates)
        if not candidates:
            self.no_frontier_count += 1
            if self.maybe_send_extra_exploration_goal():
                return
            self.publish_status('no_frontiers', no_frontier_count=self.no_frontier_count)
            return

        self.no_frontier_count = 0
        best = max(candidates, key=lambda candidate: candidate.score)
        self.send_goal(best, goal_kind='frontier')

    def find_frontier_candidates(self) -> list[FrontierCandidate]:
        height, width = self.map_data.shape
        resolution = self.map_info.resolution
        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y

        free_mask = self.map_data == FREE
        occupied_mask = self.map_data > OCCUPIED_THRESHOLD
        padded = np.pad(self.map_data, 1, mode='constant', constant_values=OCCUPIED_THRESHOLD)
        unknown_neighbor = (
            (padded[0:-2, 1:-1] == UNKNOWN) |
            (padded[2:, 1:-1] == UNKNOWN) |
            (padded[1:-1, 0:-2] == UNKNOWN) |
            (padded[1:-1, 2:] == UNKNOWN)
        )

        near_wall = occupied_mask.copy()
        dilate_r = max(1, int(math.ceil(self.wall_clearance / resolution)))
        for _ in range(dilate_r):
            p = np.pad(near_wall, 1, mode='constant', constant_values=False)
            near_wall = (
                p[1:-1, 1:-1] |
                p[0:-2, 1:-1] |
                p[2:, 1:-1] |
                p[1:-1, 0:-2] |
                p[1:-1, 2:]
            )

        frontier_mask = free_mask & unknown_neighbor & ~near_wall
        frontier_ys, frontier_xs = np.where(frontier_mask)
        if len(frontier_ys) == 0:
            return []

        min_cells = max(1, int(self.min_frontier_size / resolution))
        visited: set[tuple[int, int]] = set()
        candidates: list[FrontierCandidate] = []

        for i in range(len(frontier_ys)):
            cell = (int(frontier_ys[i]), int(frontier_xs[i]))
            if cell in visited:
                continue

            cluster = []
            queue = deque([cell])
            visited.add(cell)
            while queue:
                cy, cx = queue.popleft()
                wx = origin_x + (cx + 0.5) * resolution
                wy = origin_y + (cy + 0.5) * resolution
                cluster.append((wx, wy))
                for dy in (-1, 0, 1):
                    for dx in (-1, 0, 1):
                        if dy == 0 and dx == 0:
                            continue
                        ny, nx = cy + dy, cx + dx
                        neighbor = (ny, nx)
                        if (
                            0 <= ny < height and 0 <= nx < width and
                            neighbor not in visited and frontier_mask[ny, nx]
                        ):
                            visited.add(neighbor)
                            queue.append(neighbor)

            if len(cluster) < min_cells:
                continue

            candidate = self.score_cluster(cluster)
            if candidate is not None:
                candidates.append(candidate)

        return candidates

    def maybe_send_extra_exploration_goal(self) -> bool:
        if not self.extra_exploration_enabled or self.extra_exploration_goal_limit <= 0:
            return False
        if self.extra_exploration_goals_completed >= self.extra_exploration_goal_limit:
            if not self.extra_exploration_finished_reported:
                self.extra_exploration_finished_reported = True
                self.publish_status(
                    'extra_exploration_completed',
                    extra_exploration_goals_completed=self.extra_exploration_goals_completed,
                    extra_exploration_goal_limit=self.extra_exploration_goal_limit,
                )
            return False
        if self.extra_exploration_goal_attempts >= self.extra_exploration_max_attempts:
            if not self.extra_exploration_finished_reported:
                self.extra_exploration_finished_reported = True
                self.publish_status(
                    'extra_exploration_stopped',
                    reason='attempt limit reached',
                    extra_exploration_goal_attempts=self.extra_exploration_goal_attempts,
                    extra_exploration_max_attempts=self.extra_exploration_max_attempts,
                    extra_exploration_goals_completed=self.extra_exploration_goals_completed,
                )
            return False
        if self.semantic_vector_store is None:
            self.semantic_vector_store = SemanticVectorStore(self.semantic_vector_store_path)

        candidates = self.find_extra_exploration_candidates()
        if not candidates:
            if self.no_frontier_count == 1 or self.no_frontier_count % 10 == 0:
                self.publish_status(
                    'extra_exploration_no_goals',
                    extra_exploration_goals_completed=self.extra_exploration_goals_completed,
                    extra_exploration_goal_limit=self.extra_exploration_goal_limit,
                    coverage_radius=round(self.extra_exploration_radius, 3),
                    coverage_min_views=self.extra_exploration_min_views,
                )
            return False

        if not self.extra_exploration_started_reported:
            self.extra_exploration_started_reported = True
            self.publish_status(
                'extra_exploration_started',
                extra_exploration_goal_limit=self.extra_exploration_goal_limit,
                coverage_radius=round(self.extra_exploration_radius, 3),
                coverage_min_views=self.extra_exploration_min_views,
            )

        best = max(candidates, key=lambda candidate: candidate.score)
        self.send_goal(best, goal_kind='extra_exploration')
        return True

    def find_extra_exploration_candidates(self) -> list[CoverageCandidate]:
        if self.semantic_vector_store is None or self.map_data is None or self.map_info is None:
            return []
        height, width = self.map_data.shape
        resolution = float(self.map_info.resolution)
        if resolution <= 0.0:
            return []

        origin_x = self.map_info.origin.position.x
        origin_y = self.map_info.origin.position.y
        free_mask = self.map_data == FREE
        occupied_mask = self.map_data > OCCUPIED_THRESHOLD
        near_wall = occupied_mask.copy()
        dilate_r = max(1, int(math.ceil(self.wall_clearance / resolution)))
        for _ in range(dilate_r):
            p = np.pad(near_wall, 1, mode='constant', constant_values=False)
            near_wall = (
                p[1:-1, 1:-1] |
                p[0:-2, 1:-1] |
                p[2:, 1:-1] |
                p[1:-1, 0:-2] |
                p[1:-1, 2:]
            )

        spacing_cells = max(1, int(round(self.extra_exploration_goal_spacing / resolution)))
        offset = max(0, spacing_cells // 2)
        candidates: list[CoverageCandidate] = []
        for cy in range(offset, height, spacing_cells):
            for cx in range(offset, width, spacing_cells):
                if not free_mask[cy, cx] or near_wall[cy, cx]:
                    continue
                wx = origin_x + (cx + 0.5) * resolution
                wy = origin_y + (cy + 0.5) * resolution
                distance = math.hypot(wx - self.robot_x, wy - self.robot_y)
                if distance < self.min_goal_distance:
                    continue
                revisit_penalty = self.extra_exploration_revisit_penalty(wx, wy)
                blacklist_penalty = self.extra_exploration_blacklist_penalty(wx, wy)
                if revisit_penalty >= 100.0 or blacklist_penalty >= 100.0:
                    continue

                view_buckets = self.semantic_vector_store.area_view_bucket_count(
                    wx,
                    wy,
                    self.extra_exploration_radius,
                    self.extra_exploration_min_views,
                )
                if view_buckets >= self.extra_exploration_min_views:
                    continue
                missing_views = self.extra_exploration_min_views - view_buckets
                score = (
                    missing_views * 8.0
                    + min(distance, 5.0) * 0.2
                    - revisit_penalty
                    - blacklist_penalty
                )
                candidates.append(CoverageCandidate(
                    x=wx,
                    y=wy,
                    distance=distance,
                    score=score,
                    view_buckets=view_buckets,
                    missing_views=missing_views,
                    revisit_penalty=revisit_penalty,
                    blacklist_penalty=blacklist_penalty,
                ))
        return candidates

    def extra_exploration_revisit_penalty(self, x: float, y: float) -> float:
        penalty = 0.0
        recent_targets = (
            self.extra_exploration_targets_visited[-40:]
            + self.targets_visited[-40:]
        )
        hard_radius = max(0.45, self.extra_exploration_goal_spacing * 0.8)
        soft_radius = max(self.extra_exploration_radius, hard_radius + 0.3)
        for vx, vy in recent_targets:
            distance = math.hypot(x - vx, y - vy)
            if distance < hard_radius:
                return 100.0
            if distance < soft_radius:
                penalty += (soft_radius - distance) * 5.0
        return penalty

    def extra_exploration_blacklist_penalty(self, x: float, y: float) -> float:
        penalty = 0.0
        hard_radius = max(0.55, self.extra_exploration_goal_spacing * 0.8)
        soft_radius = max(1.2, self.extra_exploration_radius + 0.4)
        for bx, by in self.extra_exploration_blacklisted[-40:]:
            distance = math.hypot(x - bx, y - by)
            if distance < hard_radius:
                return 100.0
            if distance < soft_radius:
                penalty += (soft_radius - distance) * 8.0
        return penalty

    def score_cluster(self, cluster: list[tuple[float, float]]) -> FrontierCandidate | None:
        fx, fy = nearest_point_to_centroid(cluster)
        distance = math.hypot(fx - self.robot_x, fy - self.robot_y)
        if distance < self.min_goal_distance:
            return None

        revisit_penalty = self.revisit_penalty(fx, fy)
        blacklist_penalty = self.blacklist_penalty(fx, fy)
        if blacklist_penalty >= 100.0:
            return None

        size_score = math.sqrt(len(cluster)) * 1.25
        distance_penalty = distance * 1.0
        bias_score = self.bias_score(fx, fy)
        score = size_score - distance_penalty - revisit_penalty - blacklist_penalty + bias_score

        return FrontierCandidate(
            x=fx,
            y=fy,
            size_cells=len(cluster),
            distance=distance,
            score=score,
            bias_score=bias_score,
            revisit_penalty=revisit_penalty,
            blacklist_penalty=blacklist_penalty,
        )

    def revisit_penalty(self, x: float, y: float) -> float:
        penalty = 0.0
        for vx, vy in self.targets_visited[-40:]:
            distance = math.hypot(x - vx, y - vy)
            if distance < 0.45:
                return 50.0
            if distance < 1.2:
                penalty += (1.2 - distance) * 4.0
        return penalty

    def blacklist_penalty(self, x: float, y: float) -> float:
        penalty = 0.0
        for bx, by in self.blacklisted[-40:]:
            distance = math.hypot(x - bx, y - by)
            if distance < 0.7:
                return 100.0
            if distance < 1.4:
                penalty += (1.4 - distance) * 8.0
        return penalty

    def bias_score(self, x: float, y: float) -> float:
        if self.bias is None:
            return 0.0

        weight = float(self.bias.get('weight', 1.0))
        if self.bias['type'] == 'direction':
            requested = math.radians(float(self.bias['heading_deg']))
            target_heading = math.atan2(y - self.robot_y, x - self.robot_x)
            alignment = math.cos(normalize_angle(target_heading - requested))
            forward_projection = max(
                0.0,
                (x - self.robot_x) * math.cos(requested) +
                (y - self.robot_y) * math.sin(requested),
            )
            return weight * (4.0 * max(0.0, alignment) + 0.25 * min(forward_projection, 4.0))

        if self.bias['type'] == 'point':
            distance_to_point = math.hypot(x - self.bias['x'], y - self.bias['y'])
            return weight * (6.0 / (1.0 + distance_to_point))

        return 0.0

    def send_goal(
        self,
        candidate: FrontierCandidate | CoverageCandidate,
        *,
        goal_kind: str = 'frontier',
    ) -> None:
        if not self.nav_client.wait_for_server(timeout_sec=0.1):
            self.publish_status('waiting_for_nav2', candidates=len(self.find_frontier_candidates()))
            return

        yaw = math.atan2(candidate.y - self.robot_y, candidate.x - self.robot_x)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(candidate.x)
        pose.pose.position.y = float(candidate.y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.goal_sequence += 1
        goal_id = self.goal_sequence
        self.active_goal_id = goal_id
        self.current_target = (candidate.x, candidate.y)
        target = self.current_target
        self.current_goal_kind = goal_kind
        self.goal_pending = True
        self.goal_started_at = time.monotonic()
        self.last_goal_sent_at = self.goal_started_at
        self.within_arrival_tolerance_reported = False
        if goal_kind == 'extra_exploration':
            self.extra_exploration_goal_attempts += 1

        send_future = self.nav_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        send_future.add_done_callback(
            lambda future: self.goal_response_callback(future, goal_id, target))

        if goal_kind == 'extra_exploration':
            self.publish_status(
                'extra_exploration_goal_sent',
                goal_id=self.goal_sequence,
                target=self.target_dict(self.current_target),
                coverage={
                    'score': round(candidate.score, 3),
                    'distance': round(candidate.distance, 3),
                    'view_buckets': candidate.view_buckets,
                    'missing_views': candidate.missing_views,
                    'revisit_penalty': round(candidate.revisit_penalty, 3),
                    'blacklist_penalty': round(candidate.blacklist_penalty, 3),
                },
                extra_exploration_goal_attempts=self.extra_exploration_goal_attempts,
                extra_exploration_goal_limit=self.extra_exploration_goal_limit,
            )
        else:
            self.publish_status(
                'goal_sent',
                goal_id=self.goal_sequence,
                target=self.target_dict(self.current_target),
                frontier={
                    'score': round(candidate.score, 3),
                    'size_cells': candidate.size_cells,
                    'distance': round(candidate.distance, 3),
                    'bias_score': round(candidate.bias_score, 3),
                    'revisit_penalty': round(candidate.revisit_penalty, 3),
                    'blacklist_penalty': round(candidate.blacklist_penalty, 3),
                },
                candidates=self.last_candidate_count,
            )

    def goal_response_callback(
        self,
        future,
        goal_id: int,
        target: tuple[float, float] | None,
    ) -> None:
        if goal_id != self.active_goal_id:
            return
        self.goal_pending = False
        try:
            goal_handle = future.result()
        except Exception as exc:  # noqa: BLE001 - rclpy futures may raise transport errors.
            self.publish_status('goal_error', error=str(exc), target=self.target_dict(self.current_target))
            self.blacklist_current_target(self.current_goal_kind)
            self.clear_current_goal()
            return

        if not goal_handle.accepted:
            self.publish_status('goal_rejected', target=self.target_dict(self.current_target))
            self.blacklist_current_target(self.current_goal_kind)
            self.clear_current_goal()
            return

        self.current_goal_handle = goal_handle
        self.publish_status('goal_accepted', target=self.target_dict(self.current_target))
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda result: self.goal_result_callback(result, goal_id, target))

    def goal_result_callback(
        self,
        future,
        goal_id: int,
        target: tuple[float, float] | None,
    ) -> None:
        if goal_id != self.active_goal_id:
            return
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - rclpy futures may raise transport errors.
            self.blacklist_current_target(self.current_goal_kind)
            self.publish_status('goal_error', error=str(exc), target=self.target_dict(self.current_target))
            self.clear_current_goal()
            return

        status = result.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.complete_goal_success(
                target,
                arrival_mode='nav2_succeeded',
                goal_kind=self.current_goal_kind or 'frontier',
            )
        else:
            self.blacklist_current_target(self.current_goal_kind)
            self.publish_status(
                'goal_failed',
                status_code=int(status),
                target=self.target_dict(target),
                goal_kind=self.current_goal_kind or 'frontier',
            )
            self.clear_current_goal()

    def complete_goal_success(
        self,
        target: tuple[float, float] | None,
        arrival_mode: str,
        goal_kind: str = 'frontier',
    ) -> None:
        if goal_kind == 'extra_exploration':
            if target is not None:
                self.extra_exploration_targets_visited.append(target)
            self.extra_exploration_goals_completed += 1
        elif target is not None:
            self.targets_visited.append(target)
        self.goals_succeeded += 1
        capture_requested = self.request_semantic_capture(target, goal_kind=goal_kind)
        status_state = (
            'extra_exploration_goal_succeeded'
            if goal_kind == 'extra_exploration'
            else 'goal_succeeded'
        )
        self.publish_status(
            status_state,
            target=self.target_dict(target),
            arrival_mode=arrival_mode,
            goal_kind=goal_kind,
            semantic_capture_requested=capture_requested,
            semantic_capture_request_id=self.capture_pending_request_id,
            extra_exploration_goals_completed=self.extra_exploration_goals_completed,
            extra_exploration_goal_limit=self.extra_exploration_goal_limit,
        )
        self.clear_current_goal()

    def request_semantic_capture(
        self,
        target: tuple[float, float] | None,
        *,
        goal_kind: str = 'frontier',
    ) -> bool:
        if not self.semantic_capture_enabled:
            return False
        if self.capture_request_pub.get_subscription_count() <= 0:
            return False

        request_prefix = 'extra' if goal_kind == 'extra_exploration' else 'frontier'
        request_id = f'{request_prefix}_{self.goal_sequence}_{int(time.monotonic() * 1000)}'
        reason = 'extra_exploration' if goal_kind == 'extra_exploration' else 'frontier_arrival'
        payload = {
            'type': f'{request_prefix}_goal_succeeded',
            'request_id': request_id,
            'reason': reason,
            'target': self.target_dict(target),
            'pose': self.current_pose_payload(),
            'goal_id': self.goal_sequence,
            'goal_kind': goal_kind,
            'visited_count': len(self.targets_visited),
            'extra_exploration_goals_completed': self.extra_exploration_goals_completed,
            'extra_exploration_goal_limit': self.extra_exploration_goal_limit,
            'capture_radius_hint': (
                self.extra_exploration_radius
                if goal_kind == 'extra_exploration'
                else 1.0
            ),
        }
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.capture_request_pub.publish(msg)
        self.capture_pending_request_id = request_id
        self.capture_pending_started_at = time.monotonic()
        return True

    def capture_status_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        request_id = payload.get('request_id')
        if request_id != self.capture_pending_request_id:
            return
        state = str(payload.get('state') or '')
        if state not in {'completed', 'skipped', 'failed', 'busy'}:
            return
        self.capture_pending_request_id = None
        self.capture_pending_started_at = None
        self.publish_status(
            'semantic_capture_finished',
            request_id=request_id,
            capture_state=state,
            capture_summary=payload,
        )

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback
        distance_remaining = getattr(feedback, 'distance_remaining', None)
        if distance_remaining is not None:
            self.publish_status(
                'goal_feedback',
                target=self.target_dict(self.current_target),
                distance_remaining=round(float(distance_remaining), 3),
            )

    def blacklist_current_target(self, goal_kind: str | None = None) -> None:
        if self.current_target is None:
            return
        if goal_kind == 'extra_exploration':
            self.extra_exploration_blacklisted.append(self.current_target)
            if len(self.extra_exploration_blacklisted) > 80:
                self.extra_exploration_blacklisted = self.extra_exploration_blacklisted[-80:]
            return
        self.blacklisted.append(self.current_target)
        if len(self.blacklisted) > 80:
            self.blacklisted = self.blacklisted[-80:]

    def clear_current_goal(self) -> None:
        self.current_goal_handle = None
        self.active_goal_id = None
        self.current_goal_kind = None
        self.goal_pending = False
        self.current_target = None
        self.goal_started_at = None
        self.within_arrival_tolerance_reported = False

    def stop_exploration(self) -> None:
        self.finished = True
        if self.current_goal_handle is not None:
            try:
                self.current_goal_handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
        self.cmd_vel_pub.publish(Twist())
        self.clear_current_goal()

    def voltage_is_low(self) -> bool:
        return (
            self.min_voltage > 0.0
            and self.latest_voltage is not None
            and self.latest_voltage < self.min_voltage
        )

    def destroy_node(self) -> bool:
        if rclpy.ok():
            self.stop_exploration()
        return super().destroy_node()

    def expire_bias_if_needed(self) -> None:
        if self.bias is None:
            return
        if time.monotonic() >= float(self.bias['expires_at']):
            expired = self.public_bias()
            self.bias = None
            self.publish_status('bias_expired', bias=expired)

    def public_bias(self) -> dict[str, Any] | None:
        if self.bias is None:
            return None
        result = {k: v for k, v in self.bias.items() if k != 'expires_at'}
        result['remaining_sec'] = max(0.0, float(self.bias['expires_at']) - time.monotonic())
        result['remaining_sec'] = round(result['remaining_sec'], 1)
        return result

    def target_dict(self, target: tuple[float, float] | None) -> dict[str, float] | None:
        if target is None:
            return None
        return {'x': round(float(target[0]), 3), 'y': round(float(target[1]), 3)}

    def current_pose_payload(self) -> dict[str, Any]:
        qx, qy, qz, qw = quaternion_from_yaw(self.robot_yaw)
        return {
            'frame_id': 'map',
            'position': {
                'x': float(self.robot_x),
                'y': float(self.robot_y),
                'z': 0.0,
            },
            'orientation': {
                'x': qx,
                'y': qy,
                'z': qz,
                'w': qw,
            },
            'yaw': float(self.robot_yaw),
        }

    def publish_status(self, state: str, **extra: Any) -> None:
        payload: dict[str, Any] = {
            'state': state,
            'stamp': self.get_clock().now().nanoseconds / 1e9,
            'pose': {
                'x': round(self.robot_x, 3),
                'y': round(self.robot_y, 3),
                'yaw': round(self.robot_yaw, 3),
                'source': self.pose_source,
            },
            'target': self.target_dict(self.current_target),
            'bias': self.public_bias(),
            'visited_count': len(self.targets_visited),
            'blacklisted_count': len(self.blacklisted),
            'goals_succeeded': self.goals_succeeded,
            'semantic_capture_pending': self.capture_pending_request_id,
            'current_goal_kind': self.current_goal_kind,
            'extra_exploration_enabled': self.extra_exploration_enabled,
            'extra_exploration_goal_limit': self.extra_exploration_goal_limit,
            'extra_exploration_goal_attempts': self.extra_exploration_goal_attempts,
            'extra_exploration_goals_completed': self.extra_exploration_goals_completed,
            'extra_exploration_blacklisted_count': len(self.extra_exploration_blacklisted),
            'voltage': None if self.latest_voltage is None else round(self.latest_voltage, 2),
            'min_voltage': self.min_voltage,
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        if state not in {'goal_feedback'}:
            self.get_logger().info(msg.data)
        self.status_pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description='Biased frontier explorer using Nav2')
    parser.add_argument('--duration', type=float, default=0.0,
                        help='Max duration in seconds; 0 means unlimited')
    parser.add_argument('--min-frontier-size', type=float, default=0.2,
                        help='Minimum frontier cluster size in meters')
    parser.add_argument('--goal-timeout', type=float, default=90.0,
                        help='Seconds before blacklisting an active Nav2 goal')
    parser.add_argument('--arrival-tolerance', type=float, default=0.45,
                        help='Report near-arrival within this distance while waiting for Nav2 success')
    parser.add_argument('--min-goal-distance', type=float, default=0.75,
                        help='Ignore frontier goals closer than this many meters')
    parser.add_argument('--replan-period', type=float, default=2.0,
                        help='Minimum seconds between goal submissions')
    parser.add_argument('--wall-clearance', type=float, default=0.12,
                        help='Reject frontier cells within this many meters of occupied cells')
    parser.add_argument('--semantic-capture', default='true',
                        help='Publish /semantic_capture/request after successful frontier goals')
    parser.add_argument('--semantic-capture-timeout', type=float, default=150.0,
                        help='Max seconds to wait for semantic capture before exploring again')
    parser.add_argument('--semantic-vector-store', default=None,
                        help='SQLite semantic memory used to choose post-frontier coverage goals')
    parser.add_argument('--extra-exploration', default='false',
                        help='After frontiers are exhausted, visit under-covered free-space goals')
    parser.add_argument('--extra-exploration-goals', type=int, default=3,
                        help='Maximum successful post-frontier coverage goals when enabled')
    parser.add_argument('--extra-exploration-goal-spacing', type=float, default=0.8,
                        help='Map sampling spacing in meters for post-frontier coverage goals')
    parser.add_argument('--extra-exploration-radius', type=float, default=1.0,
                        help='Coverage radius in meters used for post-frontier semantic goals')
    parser.add_argument('--extra-exploration-min-views', type=int, default=3,
                        help='Required nearby yaw buckets before an area is considered covered')
    parser.add_argument('--odom-topic', default='/odom')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--voltage-topic', default='/voltage')
    parser.add_argument('--min-voltage', type=float, default=10.5,
                        help='Stop exploration below this voltage; 0 disables the interlock')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--use-sim-time', action='store_true')
    args = parser.parse_args(remove_ros_args()[1:])
    args.semantic_capture = parse_bool(args.semantic_capture)
    args.extra_exploration = parse_bool(args.extra_exploration)

    rclpy.init()
    node = BiasedFrontierExplorer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
