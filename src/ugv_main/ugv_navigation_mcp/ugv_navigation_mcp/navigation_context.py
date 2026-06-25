from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import threading
import time
from typing import Any

import numpy as np

from ugv_navigation_mcp.grid_geometry import (
    GridInfo,
    GridSnapshot,
    detect_corners,
    map_summary,
    normalize_angle,
    validate_goal,
    yaw_from_quaternion_values,
    yaw_to_quaternion,
)


@dataclass
class TopicConfig:
    map_topic: str = '/map'
    costmap_topic: str = '/global_costmap/costmap'
    odom_topic: str = '/odom'
    depth_image_topic: str = '/oak/stereo/image_raw'
    pointcloud_topic: str = ''
    navigate_action: str = '/navigate_to_pose'
    map_frame: str = 'map'
    base_frame: str = 'base_footprint'
    use_sim_time: bool = False
    camera_hfov_deg: float = 72.0


@dataclass
class RosRuntime:
    context: 'NavigationContext'
    executor: Any
    thread: threading.Thread

    def shutdown(self) -> None:
        self.executor.shutdown()
        self.context.destroy_node()
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


class NavigationContext:
    """ROS-backed cache and operations used by the MCP tools."""

    def __init__(self, config: TopicConfig):
        import rclpy
        from nav2_msgs.action import NavigateToPose
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from tf2_ros import Buffer, TransformListener

        class _Node(Node):
            pass

        self.node = _Node(
            'navigation_mcp_context',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, bool(config.use_sim_time)),
            ],
        )
        self.config = config
        self._lock = threading.RLock()
        self._subscriptions: dict[str, Any] = {}
        self._map: GridSnapshot | None = None
        self._costmap: GridSnapshot | None = None
        self._odom_pose: dict[str, Any] | None = None
        self._depth_image: Any | None = None
        self._pointcloud: Any | None = None
        self._depth_image_topic_seen: str | None = None
        self._pointcloud_topic_seen: str | None = None
        self._navigate_action_type = NavigateToPose
        self._nav_client: Any | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.configure_topics(**asdict(config))

    def destroy_node(self) -> None:
        self.node.destroy_node()

    def create_executor(self) -> Any:
        from rclpy.executors import MultiThreadedExecutor

        executor = MultiThreadedExecutor()
        executor.add_node(self.node)
        return executor

    # ---- configuration -----------------------------------------------------

    def configure_topics(self, **kwargs: Any) -> dict[str, Any]:
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import OccupancyGrid, Odometry
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
        from rclpy.action import ActionClient
        from sensor_msgs.msg import Image, PointCloud2

        changed = False
        for key, value in kwargs.items():
            if not hasattr(self.config, key) or value is None:
                continue
            if key.endswith('_topic') or key in {'navigate_action', 'map_frame', 'base_frame'}:
                value = str(value)
            if key in {'use_sim_time'}:
                value = bool(value)
            if key in {'camera_hfov_deg'}:
                value = float(value)
            if getattr(self.config, key) != value:
                setattr(self.config, key, value)
                changed = True

        if changed or not self._subscriptions:
            for sub in self._subscriptions.values():
                try:
                    self.node.destroy_subscription(sub)
                except Exception:
                    pass
            self._subscriptions.clear()

            map_qos = QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            if self.config.map_topic:
                self._subscriptions['map'] = self.node.create_subscription(
                    OccupancyGrid,
                    self.config.map_topic,
                    self._map_cb,
                    map_qos,
                )
            if self.config.costmap_topic:
                self._subscriptions['costmap'] = self.node.create_subscription(
                    OccupancyGrid,
                    self.config.costmap_topic,
                    self._costmap_cb,
                    map_qos,
                )
            if self.config.odom_topic:
                self._subscriptions['odom'] = self.node.create_subscription(
                    Odometry,
                    self.config.odom_topic,
                    self._odom_cb,
                    qos_profile_sensor_data,
                )
            if self.config.depth_image_topic:
                self._subscriptions['depth_image'] = self.node.create_subscription(
                    Image,
                    self.config.depth_image_topic,
                    self._depth_image_cb,
                    qos_profile_sensor_data,
                )
            if self.config.pointcloud_topic:
                self._subscriptions['pointcloud'] = self.node.create_subscription(
                    PointCloud2,
                    self.config.pointcloud_topic,
                    self._pointcloud_cb,
                    qos_profile_sensor_data,
                )

        if self._nav_client is None or changed:
            if self._nav_client is not None:
                try:
                    self._nav_client.destroy()
                except Exception:
                    pass
            self._nav_client = ActionClient(
                self.node,
                NavigateToPose,
                self.config.navigate_action,
            )

        return {'configured': asdict(self.config)}

    # ---- callbacks ---------------------------------------------------------

    def _grid_from_msg(self, msg: Any) -> GridSnapshot:
        q = msg.info.origin.orientation
        info = GridInfo(
            resolution=float(msg.info.resolution),
            width=int(msg.info.width),
            height=int(msg.info.height),
            origin_x=float(msg.info.origin.position.x),
            origin_y=float(msg.info.origin.position.y),
            origin_yaw=yaw_from_quaternion_values(q.x, q.y, q.z, q.w),
        )
        data = np.array(msg.data, dtype=np.int16).reshape((info.height, info.width))
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        return GridSnapshot(
            data=data,
            info=info,
            frame_id=msg.header.frame_id or self.config.map_frame,
            stamp_sec=stamp_sec,
        )

    def _map_cb(self, msg: Any) -> None:
        with self._lock:
            self._map = self._grid_from_msg(msg)

    def _costmap_cb(self, msg: Any) -> None:
        with self._lock:
            self._costmap = self._grid_from_msg(msg)

    def _odom_cb(self, msg: Any) -> None:
        q = msg.pose.pose.orientation
        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        with self._lock:
            self._odom_pose = {
                'available': True,
                'frame_id': msg.header.frame_id or 'odom',
                'base_frame': msg.child_frame_id or self.config.base_frame,
                'x': float(msg.pose.pose.position.x),
                'y': float(msg.pose.pose.position.y),
                'yaw': yaw_from_quaternion_values(q.x, q.y, q.z, q.w),
                'source': 'odom_fallback',
                'stamp_sec': stamp_sec,
            }

    def _depth_image_cb(self, msg: Any) -> None:
        with self._lock:
            self._depth_image = msg
            self._depth_image_topic_seen = self.config.depth_image_topic

    def _pointcloud_cb(self, msg: Any) -> None:
        with self._lock:
            self._pointcloud = msg
            self._pointcloud_topic_seen = self.config.pointcloud_topic

    # ---- snapshots and status ---------------------------------------------

    def server_status(self) -> dict[str, Any]:
        with self._lock:
            return {
                'configured': asdict(self.config),
                'has_map': self._map is not None,
                'has_costmap': self._costmap is not None,
                'has_odom': self._odom_pose is not None,
                'has_depth_image': self._depth_image is not None,
                'has_pointcloud': self._pointcloud is not None,
                'node_name': self.node.get_name(),
            }

    def wait_for_map(self, timeout_sec: float = 2.0) -> GridSnapshot | None:
        deadline = time.monotonic() + max(0.0, timeout_sec)
        while time.monotonic() <= deadline:
            with self._lock:
                if self._map is not None:
                    return self._map
            time.sleep(0.05)
        with self._lock:
            return self._map

    def _snapshot(self) -> tuple[GridSnapshot | None, GridSnapshot | None]:
        with self._lock:
            return self._map, self._costmap

    def list_navigation_topics(self) -> dict[str, Any]:
        topics = self.node.get_topic_names_and_types()

        def by_type(type_name: str, keywords: tuple[str, ...] = ()) -> list[dict[str, Any]]:
            matches = []
            for name, types in topics:
                normalized = {item.replace('/', '/msg/') if '/msg/' not in item and item.count('/') == 1 else item for item in types}
                if type_name in normalized or type_name in types:
                    if not keywords or any(keyword in name.lower() for keyword in keywords):
                        matches.append({'name': name, 'types': list(types)})
            return matches

        occupancy = by_type('nav_msgs/msg/OccupancyGrid')
        return {
            'configured': asdict(self.config),
            'suggestions': {
                'map_topics': [
                    item for item in occupancy
                    if 'costmap' not in item['name'].lower()
                ],
                'costmap_topics': [
                    item for item in occupancy
                    if 'costmap' in item['name'].lower()
                ],
                'odom_topics': by_type('nav_msgs/msg/Odometry'),
                'depth_image_topics': by_type('sensor_msgs/msg/Image', ('depth', 'stereo')),
                'pointcloud_topics': by_type('sensor_msgs/msg/PointCloud2'),
            },
            'all_topic_count': len(topics),
        }

    # ---- pose --------------------------------------------------------------

    def get_robot_pose(
        self,
        frame_id: str | None = None,
        base_frame: str | None = None,
        timeout_sec: float = 0.08,
    ) -> dict[str, Any]:
        from rclpy.duration import Duration
        from rclpy.time import Time
        from tf2_ros import TransformException

        target_frame = frame_id or self.config.map_frame
        source_frame = base_frame or self.config.base_frame
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=float(timeout_sec)),
            )
            t = transform.transform.translation
            q = transform.transform.rotation
            stamp = transform.header.stamp
            return {
                'available': True,
                'frame_id': target_frame,
                'base_frame': source_frame,
                'x': float(t.x),
                'y': float(t.y),
                'yaw': yaw_from_quaternion_values(q.x, q.y, q.z, q.w),
                'source': 'tf',
                'stamp_sec': float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            }
        except TransformException as exc:
            with self._lock:
                odom = dict(self._odom_pose) if self._odom_pose is not None else None
            if odom is not None:
                odom['warning'] = (
                    f'TF lookup {target_frame}->{source_frame} failed; '
                    f'using odom fallback in frame {odom["frame_id"]}: {exc}'
                )
                return odom
            return {
                'available': False,
                'frame_id': target_frame,
                'base_frame': source_frame,
                'error': str(exc),
            }

    def _robot_xy_for_map(self) -> tuple[float, float] | None:
        pose = self.get_robot_pose(timeout_sec=0.02)
        if not pose.get('available'):
            return None
        if pose.get('frame_id') not in {self.config.map_frame, 'map'} and pose.get('source') != 'odom_fallback':
            return None
        return float(pose['x']), float(pose['y'])

    # ---- map operations ----------------------------------------------------

    def summarize_map(self, wait_timeout_sec: float = 2.0) -> dict[str, Any]:
        snapshot = self.wait_for_map(wait_timeout_sec)
        if snapshot is None:
            return {
                'available': False,
                'reason': 'no_map_received',
                'topic': self.config.map_topic,
            }
        result = map_summary(snapshot)
        result['available'] = True
        result['topic'] = self.config.map_topic
        return result

    def detect_corners(
        self,
        number_of_corners_required: int = 4,
        policy: str = 'nearest',
        min_clearance_m: float = 0.45,
        max_corner_offset_m: float = 1.2,
        use_unknown_as_wall: bool = False,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        snapshot = self.wait_for_map(wait_timeout_sec)
        if snapshot is None:
            return {
                'available': False,
                'reason': 'no_map_received',
                'topic': self.config.map_topic,
            }
        result = detect_corners(
            snapshot=snapshot,
            number_of_corners_required=number_of_corners_required,
            robot_xy=self._robot_xy_for_map(),
            policy=policy,
            min_clearance_m=min_clearance_m,
            max_corner_offset_m=max_corner_offset_m,
            use_unknown_as_wall=use_unknown_as_wall,
        )
        result['available'] = True
        result['topic'] = self.config.map_topic
        return result

    def validate_goal(
        self,
        x: float,
        y: float,
        min_clearance_m: float = 0.45,
        allow_unknown: bool = False,
        wait_timeout_sec: float = 2.0,
    ) -> dict[str, Any]:
        snapshot = self.wait_for_map(wait_timeout_sec)
        if snapshot is None:
            return {
                'valid': False,
                'reason': 'no_map_received',
                'topic': self.config.map_topic,
            }
        _map_snapshot, costmap_snapshot = self._snapshot()
        return validate_goal(
            snapshot=snapshot,
            costmap_snapshot=costmap_snapshot,
            x=x,
            y=y,
            min_clearance_m=min_clearance_m,
            robot_xy=self._robot_xy_for_map(),
            allow_unknown=allow_unknown,
        )

    # ---- Nav2 --------------------------------------------------------------

    def go_to_coordinate(
        self,
        x: float,
        y: float,
        yaw: float = 0.0,
        frame_id: str | None = None,
        wait: bool = True,
        timeout_sec: float = 60.0,
        validate: bool = True,
        min_clearance_m: float = 0.45,
    ) -> dict[str, Any]:
        from action_msgs.msg import GoalStatus
        from geometry_msgs.msg import PoseStamped

        if validate:
            validation = self.validate_goal(x=x, y=y, min_clearance_m=min_clearance_m)
            if not validation.get('valid'):
                return {
                    'accepted': False,
                    'status': 'rejected_by_validation',
                    'validation': validation,
                }
        else:
            validation = {'valid': None, 'reason': 'validation_disabled'}

        if self._nav_client is None:
            return {'accepted': False, 'status': 'no_nav_client'}

        if not self._nav_client.wait_for_server(timeout_sec=2.0):
            return {
                'accepted': False,
                'status': 'nav2_unavailable',
                'action': self.config.navigate_action,
                'validation': validation,
            }

        goal_msg = self._navigate_action_type.Goal()
        pose = PoseStamped()
        pose.header.frame_id = frame_id or self.config.map_frame
        pose.header.stamp = self.node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        qx, qy, qz, qw = yaw_to_quaternion(float(yaw))
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        goal_msg.pose = pose

        send_future = self._nav_client.send_goal_async(goal_msg)
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        while not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.03)
        if not send_future.done():
            return {
                'accepted': False,
                'status': 'send_timeout',
                'validation': validation,
            }

        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return {
                'accepted': False,
                'status': 'goal_rejected_by_nav2',
                'validation': validation,
            }

        if not wait:
            return {
                'accepted': True,
                'status': 'accepted',
                'action': self.config.navigate_action,
                'validation': validation,
            }

        result_future = goal_handle.get_result_async()
        while not result_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not result_future.done():
            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass
            return {
                'accepted': True,
                'status': 'timeout_cancel_requested',
                'validation': validation,
            }

        status_code = result_future.result().status
        status_name = {
            GoalStatus.STATUS_UNKNOWN: 'unknown',
            GoalStatus.STATUS_ACCEPTED: 'accepted',
            GoalStatus.STATUS_EXECUTING: 'executing',
            GoalStatus.STATUS_CANCELING: 'canceling',
            GoalStatus.STATUS_SUCCEEDED: 'succeeded',
            GoalStatus.STATUS_CANCELED: 'canceled',
            GoalStatus.STATUS_ABORTED: 'aborted',
        }.get(status_code, f'code_{status_code}')

        final_pose = self.get_robot_pose(timeout_sec=0.02)
        return {
            'accepted': True,
            'status': status_name,
            'status_code': int(status_code),
            'action': self.config.navigate_action,
            'validation': validation,
            'final_pose': final_pose if final_pose.get('available') else None,
        }

    # ---- depth and point cloud --------------------------------------------

    def _depth_image_to_array(self, msg: Any) -> np.ndarray:
        encoding = (msg.encoding or '').upper()
        if encoding in {'16UC1', 'MONO16'}:
            dtype = np.dtype(np.uint16)
            scale = 0.001
        elif encoding == '32FC1':
            dtype = np.dtype(np.float32)
            scale = 1.0
        elif encoding == '64FC1':
            dtype = np.dtype(np.float64)
            scale = 1.0
        else:
            raise ValueError(f'unsupported depth image encoding: {msg.encoding}')

        if bool(msg.is_bigendian) != (dtype.byteorder == '>'):
            dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')

        itemsize = dtype.itemsize
        raw = np.frombuffer(bytes(msg.data), dtype=dtype)
        row_items = int(msg.step) // itemsize
        arr = raw.reshape((int(msg.height), row_items))[:, : int(msg.width)]
        depth = arr.astype(np.float32) * float(scale)
        depth[~np.isfinite(depth)] = np.nan
        depth[depth <= 0.0] = np.nan
        return depth

    def _roi(self, arr: np.ndarray, roi: str, fraction: float = 0.35) -> tuple[np.ndarray, int, int]:
        height, width = arr.shape
        roi = (roi or 'center').lower()
        if roi == 'full':
            return arr, 0, 0
        box_w = max(1, int(width * fraction))
        box_h = max(1, int(height * fraction))
        if roi == 'left':
            x0 = width // 6
        elif roi == 'right':
            x0 = width - width // 6 - box_w
        else:
            x0 = (width - box_w) // 2
        if roi == 'top':
            y0 = height // 6
        elif roi == 'bottom':
            y0 = height - height // 6 - box_h
        else:
            y0 = (height - box_h) // 2
        x0 = max(0, min(width - box_w, x0))
        y0 = max(0, min(height - box_h, y0))
        return arr[y0:y0 + box_h, x0:x0 + box_w], x0, y0

    def _depth_image_summary(self, roi: str = 'center') -> dict[str, Any]:
        with self._lock:
            msg = self._depth_image
            topic = self._depth_image_topic_seen or self.config.depth_image_topic
        if msg is None:
            return {
                'available': False,
                'reason': 'no_depth_image_received',
                'topic': self.config.depth_image_topic,
            }

        depth = self._depth_image_to_array(msg)
        cropped, x0, y0 = self._roi(depth, roi)
        valid = cropped[np.isfinite(cropped)]
        if valid.size == 0:
            return {
                'available': True,
                'source': 'depth_image',
                'topic': topic,
                'valid_points': 0,
                'reason': 'no_valid_depth_in_roi',
            }

        local_min_index = np.nanargmin(cropped)
        min_y, min_x = np.unravel_index(local_min_index, cropped.shape)
        image_x = x0 + int(min_x)
        width = int(msg.width)
        bearing_norm = (image_x - width * 0.5) / max(width * 0.5, 1.0)
        bearing_deg = bearing_norm * (float(self.config.camera_hfov_deg) * 0.5)
        return {
            'available': True,
            'source': 'depth_image',
            'topic': topic,
            'frame_id': msg.header.frame_id,
            'encoding': msg.encoding,
            'roi': roi,
            'valid_points': int(valid.size),
            'nearest_obstacle_m': round(float(np.nanmin(valid)), 3),
            'center_depth_m': round(float(np.nanmedian(valid)), 3),
            'mean_depth_m': round(float(np.nanmean(valid)), 3),
            'obstacle_bearing_deg': round(float(bearing_deg), 2),
            'stamp_sec': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
        }

    def _pointcloud_summary(self, max_points: int = 2000) -> dict[str, Any]:
        with self._lock:
            msg = self._pointcloud
            topic = self._pointcloud_topic_seen or self.config.pointcloud_topic
        if msg is None:
            return {
                'available': False,
                'reason': 'no_pointcloud_received',
                'topic': self.config.pointcloud_topic,
            }
        try:
            from sensor_msgs_py import point_cloud2
        except ImportError as exc:
            return {
                'available': False,
                'reason': f'sensor_msgs_py unavailable: {exc}',
                'topic': topic,
            }

        points = []
        for index, point in enumerate(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)):
            if index >= max_points:
                break
            try:
                px, py, pz = float(point[0]), float(point[1]), float(point[2])
            except (TypeError, ValueError, IndexError):
                continue
            if math.isfinite(px) and math.isfinite(py) and math.isfinite(pz):
                points.append((px, py, pz))

        if not points:
            return {
                'available': True,
                'source': 'pointcloud',
                'topic': topic,
                'frame_id': msg.header.frame_id,
                'valid_points': 0,
                'reason': 'no_valid_xyz_points',
            }

        arr = np.array(points, dtype=np.float32)
        distances = np.linalg.norm(arr, axis=1)
        nearest_index = int(np.argmin(distances))
        nearest = arr[nearest_index]
        bearing = math.degrees(math.atan2(float(nearest[1]), float(nearest[0])))
        return {
            'available': True,
            'source': 'pointcloud',
            'topic': topic,
            'frame_id': msg.header.frame_id,
            'valid_points': int(arr.shape[0]),
            'nearest_obstacle_m': round(float(distances[nearest_index]), 3),
            'nearest_point': {
                'x': round(float(nearest[0]), 3),
                'y': round(float(nearest[1]), 3),
                'z': round(float(nearest[2]), 3),
            },
            'obstacle_bearing_deg': round(float(normalize_angle(math.radians(bearing)) * 180.0 / math.pi), 2),
            'stamp_sec': float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9,
        }

    def read_depth_cloud(
        self,
        source: str = 'auto',
        roi: str = 'center',
        max_points: int = 2000,
    ) -> dict[str, Any]:
        source = (source or 'auto').lower()
        if source == 'pointcloud':
            return self._pointcloud_summary(max_points=max_points)
        if source == 'depth_image':
            return self._depth_image_summary(roi=roi)
        with self._lock:
            has_pointcloud = self._pointcloud is not None
            has_depth = self._depth_image is not None
        if has_pointcloud:
            return self._pointcloud_summary(max_points=max_points)
        if has_depth:
            return self._depth_image_summary(roi=roi)
        return {
            'available': False,
            'reason': 'no_depth_image_or_pointcloud_received',
            'depth_image_topic': self.config.depth_image_topic,
            'pointcloud_topic': self.config.pointcloud_topic,
        }


def start_ros_runtime(config: TopicConfig) -> RosRuntime:
    import rclpy

    if not rclpy.ok():
        rclpy.init(args=None)
    context = NavigationContext(config)
    executor = context.create_executor()
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return RosRuntime(context=context, executor=executor, thread=thread)
