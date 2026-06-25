#!/usr/bin/env python3
"""Reliable semantic-object navigation: fused object lookup + standoff + verify.

Flow on a /semantic_nav/go_to_object request:
  1. Resolve fused semantic object (clustered observations) for the label.
  2. Pause frontier exploration.
  3. Sample standoff candidates around the estimated object position.
  4. Costmap-validate candidates; pick the closest reachable one.
  5. Send NavigateToPose; on abort, fall through to next candidate.
  6. On Nav2 success, rotate to face object and optionally run a camera
     verification pass via the OpenAI vision API.
  7. Publish a clear final state and resume exploration.

The request topic accepts either a bare label string or a JSON object such as:
  {"label": "chair"}                      # simplest form
  {"label": "potted plant", "verify": false}
  {"label": "sofa", "match": "best"}      # match kept for compatibility
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import math
import os
import re
import threading
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from ugv_semantic_navigation.semantic_vector_store import (
    DEFAULT_CAMERA_HFOV,
    SemanticVectorStore,
    default_store_path as default_vector_store_path,
    normalize_label,
    optional_float,
)

try:
    from PIL import Image as PILImage
except ImportError:
    PILImage = None


def default_store_path() -> Path:
    return Path(__file__).resolve().parent.parent / 'data' / 'objects.json'


def yaw_to_quat(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def yaw_from_quaternion(q: Any) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class SemanticNavNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__(
            'semantic_nav_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time)],
        )
        self.base_frame = args.base_frame
        self.store_path = Path(args.store)
        self.vector_store = SemanticVectorStore(
            args.vector_store,
            camera_horizontal_fov=float(args.camera_horizontal_fov),
        )
        self.standoff_radius = float(args.standoff_radius)
        self.standoff_candidates = max(4, int(args.standoff_candidates))
        self.cluster_radius = float(args.cluster_radius)
        self.exclude_fallback = bool(args.exclude_fallback)
        self.camera_hfov = float(args.camera_horizontal_fov)
        self.verify_default = bool(args.verify)
        self.openai_api_key = os.environ.get('OPENAI_API_KEY')
        self.openai_model = args.vision_model
        self.openai_timeout = float(args.openai_timeout)
        self.min_voltage = max(0.0, float(args.min_voltage))
        self.latest_voltage: float | None = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.status_pub = self.create_publisher(String, '/semantic_nav/status', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pause_pub = self.create_publisher(String, '/semantic_explorer/pause', 10)
        self.resume_pub = self.create_publisher(String, '/semantic_explorer/resume', 10)

        self.request_sub = self.create_subscription(
            String, '/semantic_nav/go_to_object', self.request_callback, 10)
        self.voltage_sub = self.create_subscription(
            Float32, args.voltage_topic, self._voltage_cb, 10)

        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.costmap: OccupancyGrid | None = None
        self.costmap_lock = threading.Lock()
        self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self._costmap_cb, costmap_qos)

        self.latest_image: Image | None = None
        self.image_lock = threading.Lock()
        self.create_subscription(
            Image, args.image_topic, self._image_cb, qos_profile_sensor_data)

        self._busy = False
        self._busy_lock = threading.Lock()
        self.get_logger().info(
            f'Semantic nav ready (store={self.store_path}, vector={self.vector_store.db_path}, '
            f'verify={self.verify_default}, exclude_fallback={self.exclude_fallback})'
        )

    # ---- subscriptions ------------------------------------------------------

    def _costmap_cb(self, msg: OccupancyGrid) -> None:
        with self.costmap_lock:
            self.costmap = msg

    def _image_cb(self, msg: Image) -> None:
        with self.image_lock:
            self.latest_image = msg

    def _voltage_cb(self, msg: Float32) -> None:
        self.latest_voltage = float(msg.data)

    # ---- request handling ---------------------------------------------------

    def request_callback(self, msg: String) -> None:
        if self.min_voltage > 0.0 and self.latest_voltage is None:
            self.publish_status('goal_rejected_waiting_for_voltage', raw_request=msg.data)
            return
        if self.voltage_is_low():
            self.publish_status(
                'goal_rejected_low_voltage',
                raw_request=msg.data,
                voltage=round(float(self.latest_voltage), 2),
                min_voltage=self.min_voltage,
            )
            self.cmd_vel_pub.publish(Twist())
            return
        with self._busy_lock:
            if self._busy:
                self.publish_status('busy', raw_request=msg.data)
                return
            self._busy = True

        threading.Thread(target=self._handle_request, args=(msg.data,), daemon=True).start()

    def _handle_request(self, raw: str) -> None:
        try:
            self._handle_request_inner(raw)
        finally:
            with self._busy_lock:
                self._busy = False
            self._publish_resume()

    def _handle_request_inner(self, raw: str) -> None:
        request_id = f'nav_{uuid.uuid4().hex[:10]}'
        payload, label, verify = self._parse_request(raw)
        if not label:
            self.publish_status('request_error', request_id=request_id, error='missing label', raw=raw)
            return

        self._publish_pause(request_id, label)
        time.sleep(0.6)  # let explorer see pause and cancel its active Nav2 goal
        self.publish_status('resolving', request_id=request_id, label=label)

        excludes: tuple[str, ...] = ('world_landmark_fallback',) if self.exclude_fallback else ()
        clusters = self.vector_store.get_fused_objects(
            label,
            cluster_radius=self.cluster_radius,
            exclude_sources=excludes,
        )
        if not clusters:
            self.publish_status(
                'goal_failed_no_trusted_semantic_target',
                request_id=request_id,
                label=label,
                reason='no fused observations with estimated position',
            )
            return

        cluster = clusters[0]
        object_x = float(cluster['object_x'])
        object_y = float(cluster['object_y'])
        self.publish_status(
            'object_resolved',
            request_id=request_id,
            label=label,
            object_x=round(object_x, 3),
            object_y=round(object_y, 3),
            n_observations=cluster['n_observations'],
            cluster_score=cluster['cluster_score'],
            best_observation_id=cluster['best_observation'].get('id'),
        )

        robot = self._lookup_robot_pose()
        if robot is None:
            self.publish_status('goal_failed_no_pose', request_id=request_id, label=label)
            return

        candidates = self._standoff_candidates(robot, object_x, object_y)
        if not candidates:
            self.publish_status('goal_failed_unreachable',
                                request_id=request_id, label=label,
                                reason='no costmap-valid standoff candidates')
            return

        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.publish_status('waiting_for_nav2', request_id=request_id, label=label)
            return

        success_pose = None
        for index, cand in enumerate(candidates):
            self.publish_status(
                'goal_attempt',
                request_id=request_id, label=label,
                candidate_index=index, candidate_total=len(candidates),
                pose={'x': round(cand['x'], 3), 'y': round(cand['y'], 3),
                      'yaw': round(cand['yaw'], 3), 'frame_id': 'map'},
            )
            ok = self._send_blocking(cand)
            if ok:
                success_pose = cand
                break

        if success_pose is None:
            self.publish_status('goal_failed_unreachable', request_id=request_id, label=label,
                                reason='all candidates aborted')
            return

        # Rotate to face object precisely after Nav2 settles.
        self._rotate_to_face(object_x, object_y, timeout=4.0)

        verified, detail = self._verify_visually(label) if verify else (None, 'verification_disabled')
        if verified is True:
            state = 'goal_succeeded_verified'
        elif verified is False:
            state = 'goal_succeeded_not_visible'
        else:
            state = 'goal_succeeded_oriented'

        self.publish_status(
            state,
            request_id=request_id, label=label,
            object_x=round(object_x, 3), object_y=round(object_y, 3),
            standoff_pose={'x': round(success_pose['x'], 3),
                           'y': round(success_pose['y'], 3),
                           'yaw': round(success_pose['yaw'], 3)},
            verification=detail,
        )

    @staticmethod
    def _parse_request(raw: str) -> tuple[dict[str, Any], str, bool]:
        raw = (raw or '').strip()
        verify_default = True
        try:
            payload = json.loads(raw)
            if isinstance(payload, dict):
                label = normalize_label(str(payload.get('label') or payload.get('object') or ''))
                verify = bool(payload.get('verify', verify_default))
                return payload, label, verify
            if isinstance(payload, str):
                return {}, normalize_label(payload), verify_default
        except (json.JSONDecodeError, TypeError):
            pass
        return {}, normalize_label(raw), verify_default

    # ---- standoff sampling --------------------------------------------------

    def _standoff_candidates(self, robot: dict[str, float], obj_x: float, obj_y: float) -> list[dict[str, float]]:
        n = self.standoff_candidates
        radii = [self.standoff_radius, self.standoff_radius + 0.4,
                 self.standoff_radius + 0.8, self.standoff_radius + 1.2]
        cands: list[dict[str, float]] = []
        seen: set[tuple[int, int]] = set()
        base = math.atan2(robot['y'] - obj_y, robot['x'] - obj_x)
        for radius in radii:
            for i in range(n):
                offset = (i / 2 + 1) * (math.pi * 2 / n) * (1 if i % 2 == 0 else -1) if i > 0 else 0.0
                theta = normalize_angle(base + offset)
                sx = obj_x + radius * math.cos(theta)
                sy = obj_y + radius * math.sin(theta)
                key = (int(sx * 5), int(sy * 5))
                if key in seen:
                    continue
                seen.add(key)
                if not self._costmap_free(sx, sy):
                    continue
                yaw = math.atan2(obj_y - sy, obj_x - sx)
                cost = math.hypot(sx - robot['x'], sy - robot['y']) + (radius - self.standoff_radius) * 0.3
                cands.append({'x': sx, 'y': sy, 'yaw': yaw, 'cost': cost, 'radius': radius})
            if len(cands) >= 4:
                break
        cands.sort(key=lambda c: c['cost'])
        return cands[:max(8, n)]

    def _costmap_free(self, x: float, y: float) -> bool:
        with self.costmap_lock:
            grid = self.costmap
        if grid is None:
            # Be permissive when no costmap yet — Nav2 can still reject.
            return True
        info = grid.info
        gx = int((x - info.origin.position.x) / info.resolution)
        gy = int((y - info.origin.position.y) / info.resolution)
        if gx < 0 or gy < 0 or gx >= info.width or gy >= info.height:
            return False
        idx = gy * info.width + gx
        if idx >= len(grid.data):
            return False
        value = grid.data[idx]
        # -1 unknown (skip), 0 free, 1-98 inflation, 99 inscribed, 100 lethal.
        # Be permissive: only reject inscribed/lethal cells.
        return 0 <= value < 99

    # ---- navigation ---------------------------------------------------------

    def _send_blocking(self, cand: dict[str, float], timeout: float = 45.0) -> bool:
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(cand['x'])
        ps.pose.position.y = float(cand['y'])
        qx, qy, qz, qw = yaw_to_quat(float(cand['yaw']))
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        goal.pose = ps

        send_future = self.nav_client.send_goal_async(goal)
        deadline = time.monotonic() + timeout
        while not send_future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not send_future.done():
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        while not result_future.done() and time.monotonic() < deadline:
            if self.voltage_is_low():
                try:
                    handle.cancel_goal_async()
                except Exception:  # noqa: BLE001
                    pass
                self.cmd_vel_pub.publish(Twist())
                return False
            time.sleep(0.1)
        if not result_future.done():
            try:
                handle.cancel_goal_async()
            except Exception:  # noqa: BLE001
                pass
            return False
        try:
            result = result_future.result()
        except Exception:  # noqa: BLE001
            return False
        return result.status == GoalStatus.STATUS_SUCCEEDED

    def _rotate_to_face(self, obj_x: float, obj_y: float, timeout: float = 4.0) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.voltage_is_low():
                self.cmd_vel_pub.publish(Twist())
                return
            robot = self._lookup_robot_pose()
            if robot is None:
                time.sleep(0.05)
                continue
            target_yaw = math.atan2(obj_y - robot['y'], obj_x - robot['x'])
            err = normalize_angle(target_yaw - robot['yaw'])
            if abs(err) < 0.08:
                self.cmd_vel_pub.publish(Twist())
                return
            twist = Twist()
            mag = max(0.18, min(0.6, abs(err) * 1.4))
            twist.angular.z = mag if err > 0 else -mag
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self.cmd_vel_pub.publish(Twist())

    def voltage_is_low(self) -> bool:
        return (
            self.min_voltage > 0.0
            and self.latest_voltage is not None
            and self.latest_voltage < self.min_voltage
        )

    # ---- visual verification ------------------------------------------------

    def _verify_visually(self, label: str) -> tuple[bool | None, dict[str, Any]]:
        if not self.openai_api_key:
            return None, {'reason': 'no_openai_key'}
        if PILImage is None:
            return None, {'reason': 'no_pillow'}
        # Wait briefly for a fresh image after rotation.
        time.sleep(0.4)
        with self.image_lock:
            msg = self.latest_image
        if msg is None:
            return None, {'reason': 'no_image'}
        try:
            jpeg = self._image_to_jpeg(msg)
        except Exception as exc:  # noqa: BLE001
            return None, {'reason': f'encode_failed:{exc}'}
        try:
            visible, detected = self._call_vision(label, jpeg)
        except Exception as exc:  # noqa: BLE001
            return None, {'reason': f'openai_failed:{exc}'}
        return visible, {'detected_labels': detected, 'requested': label}

    @staticmethod
    def _image_to_jpeg(msg: Image, max_width: int = 384, quality: int = 70) -> bytes:
        encoding = msg.encoding.lower()
        channels = {'rgb8': 3, 'bgr8': 3, 'rgba8': 4, 'bgra8': 4, 'mono8': 1}.get(encoding)
        if channels is None:
            raise ValueError(f'unsupported encoding: {msg.encoding}')
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        rows = raw.reshape((int(msg.height), int(msg.step)))
        active = rows[:, :int(msg.width) * channels].reshape(int(msg.height), int(msg.width), channels)
        if encoding == 'rgb8' or encoding == 'rgba8':
            rgb = active[:, :, :3]
        elif encoding == 'bgr8' or encoding == 'bgra8':
            rgb = active[:, :, [2, 1, 0]]
        else:
            mono = active[:, :, 0]
            rgb = np.repeat(mono[:, :, None], 3, axis=2)
        pil = PILImage.fromarray(rgb.copy(), mode='RGB')
        if pil.width > max_width:
            ratio = max_width / float(pil.width)
            pil = pil.resize((max_width, max(1, int(pil.height * ratio))))
        buffer = io.BytesIO()
        pil.save(buffer, format='JPEG', quality=quality, optimize=True)
        return buffer.getvalue()

    def _call_vision(self, label: str, jpeg: bytes) -> tuple[bool, list[str]]:
        b64 = base64.b64encode(jpeg).decode('ascii')
        data_url = f'data:image/jpeg;base64,{b64}'
        payload = {
            'model': self.openai_model,
            'input': [
                {'role': 'system', 'content': [{'type': 'input_text', 'text':
                    'You are a robot visibility verifier. Decide if the requested object '
                    'or a clear synonym is currently visible in the image. Return strict JSON.'}]},
                {'role': 'user', 'content': [
                    {'type': 'input_text', 'text':
                        f'Is a "{label}" clearly visible in this camera frame? '
                        'Reply with the JSON schema.'},
                    {'type': 'input_image', 'image_url': data_url, 'detail': 'low'},
                ]},
            ],
            'text': {'format': {
                'type': 'json_schema', 'name': 'visibility_check', 'strict': True,
                'schema': {
                    'type': 'object', 'additionalProperties': False,
                    'required': ['visible', 'detected_labels', 'reason'],
                    'properties': {
                        'visible': {'type': 'boolean'},
                        'detected_labels': {'type': 'array', 'items': {'type': 'string'}},
                        'reason': {'type': 'string'},
                    },
                },
            }},
            'temperature': 0.0,
            'max_output_tokens': 300,
        }
        request = urllib.request.Request(
            'https://api.openai.com/v1/responses',
            data=json.dumps(payload).encode('utf-8'),
            method='POST',
            headers={
                'Authorization': f'Bearer {self.openai_api_key}',
                'Content-Type': 'application/json',
            },
        )
        with urllib.request.urlopen(request, timeout=self.openai_timeout) as response:
            data = json.loads(response.read().decode('utf-8'))
        text = data.get('output_text') or ''
        if not text:
            chunks = []
            for item in data.get('output', []) or []:
                for content in item.get('content', []) or []:
                    if content.get('text'):
                        chunks.append(str(content['text']))
            text = ''.join(chunks)
        result = json.loads(text)
        return bool(result.get('visible', False)), [str(x) for x in result.get('detected_labels', [])]

    # ---- helpers ------------------------------------------------------------

    def _lookup_robot_pose(self) -> dict[str, float] | None:
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', self.base_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.3))
        except TransformException:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return {'x': float(t.x), 'y': float(t.y), 'yaw': yaw_from_quaternion(r)}

    def _publish_pause(self, request_id: str, label: str) -> None:
        msg = String()
        msg.data = json.dumps({'reason': 'semantic_nav_active', 'request_id': request_id, 'label': label})
        self.pause_pub.publish(msg)

    def _publish_resume(self) -> None:
        msg = String()
        msg.data = json.dumps({'reason': 'semantic_nav_complete'})
        self.resume_pub.publish(msg)

    def publish_status(self, state: str, **extra: Any) -> None:
        payload = {
            'state': state,
            'stamp': self.get_clock().now().nanoseconds / 1e9,
            'store': str(self.store_path),
            'vector_store': str(self.vector_store.db_path),
            'voltage': None if self.latest_voltage is None else round(self.latest_voltage, 2),
            'min_voltage': self.min_voltage,
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)


def main() -> None:
    parser = argparse.ArgumentParser(description='Semantic object navigation node')
    parser.add_argument('--store', type=Path, default=default_store_path())
    parser.add_argument('--image-topic', default='/camera/image')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--voltage-topic', default='/voltage')
    parser.add_argument('--min-voltage', type=float, default=10.5,
                        help='Refuse or cancel semantic navigation below this voltage; 0 disables it')
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--vector-store', type=Path, default=default_vector_store_path())
    parser.add_argument('--standoff-radius', type=float, default=1.1)
    parser.add_argument('--standoff-candidates', type=int, default=8)
    parser.add_argument('--cluster-radius', type=float, default=1.0)
    parser.add_argument('--camera-horizontal-fov', type=float, default=DEFAULT_CAMERA_HFOV)
    parser.add_argument('--exclude-fallback', action='store_true', default=True)
    parser.add_argument('--include-fallback', dest='exclude_fallback', action='store_false')
    parser.add_argument('--verify', action='store_true', default=True)
    parser.add_argument('--no-verify', dest='verify', action='store_false')
    parser.add_argument('--vision-model', default=os.environ.get('OPENAI_VISION_MODEL', 'gpt-4.1-mini'))
    parser.add_argument('--openai-timeout', type=float, default=30.0)
    args = parser.parse_args(remove_ros_args()[1:])

    rclpy.init()
    node = SemanticNavNode(args)
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
