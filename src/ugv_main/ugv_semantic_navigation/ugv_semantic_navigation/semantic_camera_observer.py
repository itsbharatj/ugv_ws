#!/usr/bin/env python3
"""Frontier-triggered OpenAI vision observer for semantic mapping.

This node replaces the old RGB threshold demo detector. It waits for frontier
arrival requests, rotates the robot through a small 360-degree sweep, calls an
OpenAI vision model for each novel view, and publishes structured observations
to semantic_memory_node.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import io
import json
import math
import os
import threading
import time
import urllib.error
import urllib.request
import uuid
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformException, TransformListener

from ugv_semantic_navigation.semantic_vector_store import (
    DEFAULT_EMBEDDING_MODEL,
    SemanticVectorStore,
    default_store_path as default_vector_store_path,
    normalize_label,
    optional_float,
    utc_now,
)

try:
    from PIL import Image as PILImage
except ImportError:  # pragma: no cover - launch dependency issue, handled at runtime.
    PILImage = None


DEFAULT_VISION_MODEL = os.environ.get('OPENAI_VISION_MODEL', 'gpt-4.1-mini')
CAPTURE_SOURCE = 'openai_vla'
LANDMARK_FALLBACK_SOURCE = 'world_landmark_fallback'


@dataclass
class CapturedFrame:
    msg: Image
    jpeg: bytes
    image_sha256: str
    image_hash: str
    pose: dict[str, Any]
    yaw: float
    image_path: str | None


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_quaternion(q: Any) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def compact_pose(pose: dict[str, Any]) -> dict[str, Any]:
    position = pose.get('position') if isinstance(pose.get('position'), dict) else {}
    return {
        'frame_id': pose.get('frame_id', 'map'),
        'x': round(float(position.get('x', 0.0)), 3),
        'y': round(float(position.get('y', 0.0)), 3),
        'yaw': round(float(pose.get('yaw', 0.0)), 3),
    }


def parse_bool(value: Any) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


class OpenAISemanticObserver(Node):
    """Capture semantic views at frontier arrivals with bounded OpenAI calls."""

    def __init__(self, args: argparse.Namespace):
        super().__init__(
            'semantic_camera_observer',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, args.use_sim_time)],
        )
        self.args = args
        self.vision_model = args.vision_model
        self.image_width = max(128, int(args.image_width))
        self.jpeg_quality = max(35, min(95, int(args.jpeg_quality)))
        self.capture_angles = max(1, int(args.capture_angles))
        self.capture_radius = max(0.2, float(args.capture_radius))
        self.area_captured_min_views = max(1, int(args.area_captured_min_views))
        self.min_confidence = clamp(float(args.min_confidence), 0.0, 1.0)
        self.angular_speed = max(0.05, float(args.angular_speed))
        self.yaw_tolerance = max(0.02, float(args.yaw_tolerance))
        self.settle_time = max(0.0, float(args.settle_time))
        self.capture_timeout = max(10.0, float(args.capture_timeout))
        self.camera_horizontal_fov = max(0.1, float(args.camera_horizontal_fov))
        self.max_similar_hamming = max(0, int(args.max_similar_hamming))
        self.api_key = os.environ.get(args.api_key_env)
        self.api_base = (args.api_base or os.environ.get('OPENAI_BASE_URL') or 'https://api.openai.com/v1').rstrip('/')
        self.openai_timeout = max(5.0, float(args.openai_timeout))
        self.store_image_only_captures = parse_bool(args.store_image_only_captures)
        self.landmark_fallback = parse_bool(args.landmark_fallback)
        self.landmark_max_distance = max(0.5, float(args.landmark_max_distance))
        self.landmark_fov_margin = max(0.0, float(args.landmark_fov_margin))
        self.landmark_max_objects_per_view = max(1, int(args.landmark_max_objects_per_view))
        self.landmarks = self.load_landmarks(args.landmarks)
        self.min_voltage = max(0.0, float(args.min_voltage))
        self.latest_voltage: float | None = None

        self.latest_image: Image | None = None
        self.latest_scan: LaserScan | None = None
        self.image_lock = threading.Lock()
        self.scan_lock = threading.Lock()
        self.analysis_lock = threading.Lock()
        self.analysis_thread: threading.Thread | None = None

        self.session: dict[str, Any] | None = None
        self.last_idle_status = 0.0
        self.last_status_log_by_state: dict[str, float] = {}

        self.vector_store = SemanticVectorStore(
            args.vector_store,
            embedding_model=args.embedding_model,
            api_key=self.api_key,
        )
        self.image_dir = Path(args.image_dir) if args.image_dir else None
        if self.image_dir is not None:
            self.image_dir.mkdir(parents=True, exist_ok=True)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.remember_pub = self.create_publisher(String, args.remember_topic, 10)
        self.status_pub = self.create_publisher(String, args.status_topic, 10)
        self.capture_status_pub = self.create_publisher(String, args.capture_status_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)
        self.image_sub = self.create_subscription(
            Image, args.image_topic, self.image_callback, qos_profile_sensor_data)
        self.scan_sub = self.create_subscription(
            LaserScan, args.scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.capture_request_sub = self.create_subscription(
            String, args.capture_request_topic, self.capture_request_callback, 10)
        self.voltage_sub = self.create_subscription(
            Float32, args.voltage_topic, self.voltage_callback, 10)

        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            'OpenAI semantic observer ready: request=%s model=%s angles=%d store=%s'
            % (args.capture_request_topic, self.vision_model, self.capture_angles, args.vector_store)
        )
        if not self.api_key:
            self.get_logger().warning(
                '%s is not set; frontier sweeps will still save JPEG captures%s.'
                % (
                    args.api_key_env,
                    ' and use landmark fallback' if self.landmarks and self.landmark_fallback else '',
                )
            )

    def load_landmarks(self, path_value: str | Path | None) -> list[dict[str, Any]]:
        if not path_value:
            return []
        path = Path(path_value)
        if not path.exists():
            self.get_logger().warning(f'Landmark fallback file does not exist: {path}')
            return []
        try:
            data = json.loads(path.read_text(encoding='utf-8'))
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Cannot read landmark fallback file {path}: {exc}')
            return []

        raw_landmarks = data.get('landmarks') if isinstance(data, dict) else data
        if not isinstance(raw_landmarks, list):
            self.get_logger().warning(f'Landmark fallback file has no landmarks list: {path}')
            return []

        landmarks = []
        for item in raw_landmarks:
            if not isinstance(item, dict):
                continue
            label = normalize_label(str(item.get('label') or ''))
            x = optional_float(item.get('x'))
            y = optional_float(item.get('y'))
            if not label or x is None or y is None:
                continue
            landmark = dict(item)
            landmark['label'] = str(item.get('label') or label)
            landmark['x'] = x
            landmark['y'] = y
            landmarks.append(landmark)
        self.get_logger().info(f'Loaded {len(landmarks)} semantic fallback landmarks from {path}')
        return landmarks

    def image_callback(self, msg: Image) -> None:
        with self.image_lock:
            self.latest_image = msg

    def scan_callback(self, msg: LaserScan) -> None:
        with self.scan_lock:
            self.latest_scan = msg

    def voltage_callback(self, msg: Float32) -> None:
        self.latest_voltage = float(msg.data)

    def capture_request_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.publish_status('request_error', error=f'invalid JSON: {exc}')
            return

        request_id = str(payload.get('request_id') or f'capture_{uuid.uuid4().hex[:10]}')
        if self.session is not None:
            self.publish_capture_status('busy', request_id=request_id)
            return
        if self.min_voltage > 0.0 and self.latest_voltage is None:
            self.publish_capture_status(
                'skipped', request_id=request_id, reason='waiting for voltage reading')
            return
        if self.voltage_is_low():
            self.publish_capture_status(
                'skipped',
                request_id=request_id,
                reason='low voltage',
                voltage=round(float(self.latest_voltage), 2),
                min_voltage=self.min_voltage,
            )
            return
        if PILImage is None:
            self.publish_capture_status(
                'failed',
                request_id=request_id,
                reason='Pillow is not installed',
            )
            return

        pose = self.lookup_map_pose() or self.pose_from_request(payload)
        if pose is None:
            self.publish_capture_status(
                'skipped',
                request_id=request_id,
                reason='map->base_link transform unavailable',
            )
            return

        position = pose.get('position', {})
        x = optional_float(position.get('x'))
        y = optional_float(position.get('y'))
        if (
            x is not None
            and y is not None
            and self.vector_store.area_has_min_views(
                x, y, self.capture_radius, min(self.area_captured_min_views, self.capture_angles)
            )
        ):
            self.publish_capture_status(
                'skipped',
                request_id=request_id,
                reason='area already has semantic views',
                pose=compact_pose(pose),
            )
            return

        base_yaw = float(pose.get('yaw', 0.0))
        target_yaws = [
            normalize_angle(base_yaw + (2.0 * math.pi * index / self.capture_angles))
            for index in range(self.capture_angles)
        ]
        self.session = {
            'request_id': request_id,
            'reason': payload.get('reason', 'frontier_arrival'),
            'started_at': time.monotonic(),
            'target_yaws': target_yaws,
            'index': 0,
            'state': 'rotating',
            'settled_since': None,
            'captures': [],
            'skipped_views': 0,
            'objects': 0,
            'request': payload,
            'seen_image_hashes': set(),
        }
        self.publish_capture_status(
            'started',
            request_id=request_id,
            angles=self.capture_angles,
            pose=compact_pose(pose),
        )

    def tick(self) -> None:
        if self.session is None:
            if time.monotonic() - self.last_idle_status > 5.0:
                self.publish_status('idle')
                self.last_idle_status = time.monotonic()
            return

        if self.voltage_is_low():
            request_id = self.session['request_id']
            self.stop_robot()
            self.session = None
            self.publish_capture_status(
                'failed',
                request_id=request_id,
                reason='low voltage',
                voltage=round(float(self.latest_voltage), 2),
                min_voltage=self.min_voltage,
            )
            return

        if time.monotonic() - float(self.session['started_at']) > self.capture_timeout:
            request_id = self.session['request_id']
            self.stop_robot()
            self.session = None
            self.publish_capture_status('failed', request_id=request_id, reason='capture timeout')
            return

        if self.session['state'] == 'analyzing':
            return

        pose = self.lookup_map_pose()
        if pose is None:
            self.stop_robot()
            self.publish_status('waiting_for_map_pose', request_id=self.session['request_id'])
            return

        target_yaws = self.session['target_yaws']
        index = int(self.session['index'])
        if index >= len(target_yaws):
            self.finish_session()
            return

        current_yaw = float(pose.get('yaw', 0.0))
        target_yaw = float(target_yaws[index])
        error = normalize_angle(target_yaw - current_yaw)
        if abs(error) > self.yaw_tolerance:
            self.session['settled_since'] = None
            self.rotate_toward(error)
            self.publish_status(
                'rotating',
                request_id=self.session['request_id'],
                view_index=index,
                target_yaw=round(target_yaw, 3),
                yaw_error=round(error, 3),
            )
            return

        self.stop_robot()
        if self.session['settled_since'] is None:
            self.session['settled_since'] = time.monotonic()
            return
        if time.monotonic() - float(self.session['settled_since']) < self.settle_time:
            return
        self.start_analysis_for_current_view(pose)

    def start_analysis_for_current_view(self, pose: dict[str, Any]) -> None:
        frame = self.capture_frame(pose)
        request_id = self.session['request_id'] if self.session else 'unknown'
        index = int(self.session['index']) if self.session else 0
        if frame is None:
            self.publish_status('waiting_for_image', request_id=request_id)
            return

        position = frame.pose.get('position', {})
        x = optional_float(position.get('x'))
        y = optional_float(position.get('y'))
        if self.vector_store.similar_image_seen(
            frame.image_hash,
            x,
            y,
            self.capture_radius,
            max_hamming=self.max_similar_hamming,
        ) or self.session_image_seen(frame.image_hash):
            self.publish_image_only_capture(request_id, index, frame, 'skipped similar image')
            if self.session is not None:
                self.session['skipped_views'] += 1
            self.advance_to_next_view()
            return
        self.session['seen_image_hashes'].add(frame.image_hash)

        self.session['state'] = 'analyzing'
        self.publish_status(
            'analyzing',
            request_id=request_id,
            view_index=index,
            model=self.vision_model,
            pose=compact_pose(frame.pose),
        )
        thread = threading.Thread(
            target=self.analyze_view_thread,
            args=(request_id, index, frame),
            daemon=True,
        )
        with self.analysis_lock:
            self.analysis_thread = thread
        thread.start()

    def analyze_view_thread(self, request_id: str, view_index: int, frame: CapturedFrame) -> None:
        try:
            response_data, raw_response, source = self.analyze_frame(frame)
            objects = self.prepare_objects(response_data.get('objects', []), frame, source=source)
            capture_id = f'cap_{uuid.uuid4().hex[:12]}'
            capture_payload = {
                'type': 'semantic_capture',
                'request_id': request_id,
                'capture': {
                    'id': capture_id,
                    'request_id': request_id,
                    'timestamp': utc_now(),
                    'source': source,
                    'pose': frame.pose,
                    'yaw': frame.yaw,
                    'image_sha256': frame.image_sha256,
                    'image_hash': frame.image_hash,
                    'image_path': frame.image_path,
                    'image_stamp_sec': int(frame.msg.header.stamp.sec),
                    'image_stamp_nanosec': int(frame.msg.header.stamp.nanosec),
                    'summary': str(response_data.get('scene_summary') or '').strip(),
                    'model': self.vision_model,
                    'analysis_source': source,
                    'raw_response': raw_response,
                },
                'objects': objects,
            }
            self.publish_remember(capture_payload)
            if self.session is not None and self.session.get('request_id') == request_id:
                self.session['captures'].append({
                    'view_index': view_index,
                    'capture_id': capture_id,
                    'objects': len(objects),
                    'summary': capture_payload['capture']['summary'],
                })
                self.session['objects'] += len(objects)
            self.publish_status(
                'view_published',
                request_id=request_id,
                view_index=view_index,
                capture_id=capture_id,
                objects=len(objects),
                analysis_source=source,
                image_path=frame.image_path,
            )
        except Exception as exc:  # noqa: BLE001 - API/SDK/network errors should not kill ROS.
            self.get_logger().error(f'OpenAI semantic capture failed: {exc}')
            if self.store_image_only_captures:
                self.publish_image_only_capture(request_id, view_index, frame, str(exc))
            else:
                if self.session is not None and self.session.get('request_id') == request_id:
                    self.session['captures'].append({
                        'view_index': view_index,
                        'state': 'failed',
                        'error': str(exc),
                    })
                self.publish_status(
                    'view_failed',
                    request_id=request_id,
                    view_index=view_index,
                    error=str(exc),
                )
        finally:
            if self.session is not None and self.session.get('request_id') == request_id:
                self.advance_to_next_view()

    def analyze_frame(self, frame: CapturedFrame) -> tuple[dict[str, Any], dict[str, Any], str]:
        if self.api_key:
            try:
                response_data, raw_response = self.call_openai_vision(frame)
                return response_data, raw_response, CAPTURE_SOURCE
            except Exception as exc:
                landmark_objects = self.landmark_objects_for_frame(frame)
                if landmark_objects:
                    return (
                        {
                            'scene_summary': (
                                f'OpenAI capture failed; using {len(landmark_objects)} '
                                'visible known world landmarks.'
                            ),
                            'objects': landmark_objects,
                        },
                        {'fallback_reason': str(exc), 'source': LANDMARK_FALLBACK_SOURCE},
                        LANDMARK_FALLBACK_SOURCE,
                    )
                raise

        landmark_objects = self.landmark_objects_for_frame(frame)
        if landmark_objects:
            return (
                {
                    'scene_summary': (
                        f'OPENAI_API_KEY is not set; using {len(landmark_objects)} '
                        'visible known world landmarks.'
                    ),
                    'objects': landmark_objects,
                },
                {'fallback_reason': f'{self.args.api_key_env} is not set'},
                LANDMARK_FALLBACK_SOURCE,
            )

        if self.store_image_only_captures:
            return (
                {
                    'scene_summary': f'Image-only capture; {self.args.api_key_env} is not set.',
                    'objects': [],
                },
                {'fallback_reason': f'{self.args.api_key_env} is not set'},
                'image_only',
            )
        raise RuntimeError(f'{self.args.api_key_env} is not set')

    def publish_image_only_capture(
        self,
        request_id: str,
        view_index: int,
        frame: CapturedFrame,
        error: str,
    ) -> None:
        capture_id = f'cap_{uuid.uuid4().hex[:12]}'
        capture_payload = {
            'type': 'semantic_capture',
            'request_id': request_id,
            'capture': {
                'id': capture_id,
                'request_id': request_id,
                'timestamp': utc_now(),
                'source': 'image_only',
                'pose': frame.pose,
                'yaw': frame.yaw,
                'image_sha256': frame.image_sha256,
                'image_hash': frame.image_hash,
                'image_path': frame.image_path,
                'image_stamp_sec': int(frame.msg.header.stamp.sec),
                'image_stamp_nanosec': int(frame.msg.header.stamp.nanosec),
                'summary': f'Image saved but semantic analysis failed: {error}',
                'model': self.vision_model,
                'analysis_source': 'image_only',
                'raw_response': {'error': error},
            },
            'objects': [],
        }
        self.publish_remember(capture_payload)
        if self.session is not None and self.session.get('request_id') == request_id:
            self.session['captures'].append({
                'view_index': view_index,
                'capture_id': capture_id,
                'objects': 0,
                'state': 'image_only',
                'error': error,
                'image_path': frame.image_path,
            })
        self.publish_status(
            'view_saved_image_only',
            request_id=request_id,
            view_index=view_index,
            capture_id=capture_id,
            image_path=frame.image_path,
            error=error,
        )

    def landmark_objects_for_frame(self, frame: CapturedFrame) -> list[dict[str, Any]]:
        if not self.landmark_fallback or not self.landmarks:
            return []
        position = frame.pose.get('position') if isinstance(frame.pose.get('position'), dict) else {}
        robot_x = optional_float(position.get('x'))
        robot_y = optional_float(position.get('y'))
        if robot_x is None or robot_y is None:
            return []

        half_fov = self.camera_horizontal_fov * 0.5
        candidates: list[tuple[float, dict[str, Any]]] = []
        for landmark in self.landmarks:
            lx = optional_float(landmark.get('x'))
            ly = optional_float(landmark.get('y'))
            if lx is None or ly is None:
                continue
            dx = lx - robot_x
            dy = ly - robot_y
            distance = math.hypot(dx, dy)
            if distance < 0.25 or distance > self.landmark_max_distance:
                continue
            bearing = normalize_angle(math.atan2(dy, dx) - frame.yaw)
            if abs(bearing) > half_fov + self.landmark_fov_margin:
                continue

            visible_fraction = clamp(1.0 - max(0.0, abs(bearing) - half_fov * 0.65) / max(0.01, half_fov), 0.35, 1.0)
            width_m = optional_float(landmark.get('width')) or optional_float(landmark.get('radius')) or 0.55
            height_m = optional_float(landmark.get('height')) or 0.75
            bbox_width = clamp((width_m / max(distance, 0.25)) / self.camera_horizontal_fov, 0.08, 0.55)
            bbox_height = clamp(height_m / max(distance, 0.25) / 1.2, 0.12, 0.75)
            center_x = clamp(0.5 - bearing / self.camera_horizontal_fov, 0.02, 0.98)
            center_y = clamp(float(landmark.get('image_y_center', 0.56)), 0.20, 0.86)
            bbox = {
                'x_min': round(clamp(center_x - bbox_width * 0.5, 0.0, 1.0), 3),
                'y_min': round(clamp(center_y - bbox_height * 0.5, 0.0, 1.0), 3),
                'x_max': round(clamp(center_x + bbox_width * 0.5, 0.0, 1.0), 3),
                'y_max': round(clamp(center_y + bbox_height * 0.5, 0.0, 1.0), 3),
            }
            visibility = 'mostly_visible' if visible_fraction >= 0.75 else 'partial'
            confidence = optional_float(landmark.get('confidence'))
            if confidence is None:
                confidence = 0.82
            confidence = clamp(confidence * (0.75 + 0.25 * visible_fraction), 0.0, 1.0)
            if confidence < self.min_confidence:
                continue

            aliases = landmark.get('aliases')
            if not isinstance(aliases, list):
                aliases = []
            description = str(landmark.get('description') or landmark.get('label') or '').strip()
            obj = {
                'label': str(landmark.get('label') or '').strip(),
                'aliases': [str(alias) for alias in aliases],
                'description': description,
                'confidence': round(confidence, 3),
                'visibility': visibility,
                'visible_fraction': round(visible_fraction, 3),
                'is_completely_visible': visible_fraction >= 0.88,
                'bbox': bbox,
                'distance_estimate_m': round(distance, 3),
                'distance_m': round(distance, 3),
                'distance_source': 'world_landmark_pose',
            }
            candidates.append((distance, obj))

        candidates.sort(key=lambda item: item[0])
        return [obj for _, obj in candidates[:self.landmark_max_objects_per_view]]

    def call_openai_vision(self, frame: CapturedFrame) -> tuple[dict[str, Any], dict[str, Any]]:
        image_b64 = base64.b64encode(frame.jpeg).decode('ascii')
        data_url = f'data:image/jpeg;base64,{image_b64}'
        prompt = self.vision_prompt(frame)
        payload = {
            'model': self.vision_model,
            'input': [
                {
                    'role': 'system',
                    'content': [
                        {
                            'type': 'input_text',
                            'text': (
                                'You are a robot semantic mapper. Identify stable, '
                                'navigation-useful objects and landmarks in the image. '
                                'Do not guess objects that are not visible. Return JSON '
                                'that exactly matches the schema.'
                            ),
                        }
                    ],
                },
                {
                    'role': 'user',
                    'content': [
                        {'type': 'input_text', 'text': prompt},
                        {'type': 'input_image', 'image_url': data_url, 'detail': self.args.image_detail},
                    ],
                },
            ],
            'text': {'format': self.vision_schema()},
            'temperature': 0.1,
            'max_output_tokens': self.args.max_output_tokens,
        }
        response = self.openai_post('/responses', payload)
        output_text = self.response_output_text(response)
        if not output_text:
            raise ValueError('OpenAI response did not include output_text')
        data = json.loads(output_text)
        return data, response

    def openai_post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        if not self.api_key:
            raise RuntimeError(f'{self.args.api_key_env} is not set')
        request = urllib.request.Request(
            f'{self.api_base}{path}',
            data=json.dumps(payload).encode('utf-8'),
            method='POST',
            headers={
                'Authorization': f'Bearer {self.api_key}',
                'Content-Type': 'application/json',
            },
        )
        try:
            with urllib.request.urlopen(request, timeout=self.openai_timeout) as response:
                return json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            raise RuntimeError(f'OpenAI HTTP {exc.code}: {detail}') from exc

    def vision_prompt(self, frame: CapturedFrame) -> str:
        pose = compact_pose(frame.pose)
        scan = self.scan_summary()
        return (
            'Analyze this robot camera frame for semantic mapping.\n'
            f'Robot pose in map frame: x={pose["x"]}, y={pose["y"]}, yaw={pose["yaw"]} rad.\n'
            f'Approximate lidar context: {scan}.\n'
            'Return only persistent objects or landmarks that a person might ask the robot to '
            'find later, such as furniture, appliances, boxes, plants, balls, doors, tables, '
            'chairs, shelves, signs, or distinctive room fixtures. Ignore blank walls, floor, '
            'ceiling, shadows, and tiny uncertain fragments. Merge duplicate parts of the same '
            'object in this single image. Use generic labels that work across homes and offices. '
            'For each object include a normalized bounding box, confidence from 0 to 1, whether '
            'it is fully visible, approximate distance if inferable, and a short navigation '
            'description. If no useful objects are visible, return an empty objects array.'
        )

    @staticmethod
    def vision_schema() -> dict[str, Any]:
        bbox_schema = {
            'type': 'object',
            'additionalProperties': False,
            'required': ['x_min', 'y_min', 'x_max', 'y_max'],
            'properties': {
                'x_min': {'type': 'number'},
                'y_min': {'type': 'number'},
                'x_max': {'type': 'number'},
                'y_max': {'type': 'number'},
            },
        }
        object_schema = {
            'type': 'object',
            'additionalProperties': False,
            'required': [
                'label',
                'aliases',
                'description',
                'confidence',
                'visibility',
                'visible_fraction',
                'is_completely_visible',
                'bbox',
                'distance_estimate_m',
            ],
            'properties': {
                'label': {'type': 'string'},
                'aliases': {'type': 'array', 'items': {'type': 'string'}},
                'description': {'type': 'string'},
                'confidence': {'type': 'number'},
                'visibility': {
                    'type': 'string',
                    'enum': ['full', 'mostly_visible', 'partial', 'occluded', 'uncertain'],
                },
                'visible_fraction': {'type': 'number'},
                'is_completely_visible': {'type': 'boolean'},
                'bbox': bbox_schema,
                'distance_estimate_m': {
                    'anyOf': [
                        {'type': 'number'},
                        {'type': 'null'},
                    ],
                },
            },
        }
        return {
            'type': 'json_schema',
            'name': 'robot_semantic_capture',
            'strict': True,
            'schema': {
                'type': 'object',
                'additionalProperties': False,
                'required': ['scene_summary', 'objects'],
                'properties': {
                    'scene_summary': {'type': 'string'},
                    'objects': {'type': 'array', 'items': object_schema},
                },
            },
        }

    @staticmethod
    def response_output_text(response: Any) -> str:
        if isinstance(response, dict):
            output_text = response.get('output_text')
            if output_text:
                return str(output_text)
            chunks = []
            for item in response.get('output', []) or []:
                for content in item.get('content', []) or []:
                    if content.get('type') == 'refusal':
                        raise ValueError(str(content.get('refusal') or 'OpenAI refused the request'))
                    text = content.get('text')
                    if text:
                        chunks.append(str(text))
            return ''.join(chunks)

        output_text = getattr(response, 'output_text', None)
        if output_text:
            return str(output_text)
        chunks = []
        for item in getattr(response, 'output', []) or []:
            for content in getattr(item, 'content', []) or []:
                text = getattr(content, 'text', None)
                if text:
                    chunks.append(str(text))
        return ''.join(chunks)

    def prepare_objects(
        self,
        raw_objects: list[Any],
        frame: CapturedFrame,
        *,
        source: str = CAPTURE_SOURCE,
    ) -> list[dict[str, Any]]:
        prepared: list[dict[str, Any]] = []
        for raw in raw_objects:
            if not isinstance(raw, dict):
                continue
            label = normalize_label(str(raw.get('label') or ''))
            if not label:
                continue
            confidence = optional_float(raw.get('confidence'))
            if confidence is not None and confidence < self.min_confidence:
                continue
            obj = dict(raw)
            obj['id'] = f'obs_{uuid.uuid4().hex[:12]}'
            obj['source'] = source
            lidar_distance = self.distance_from_lidar(obj.get('bbox'))
            if lidar_distance is not None:
                obj['distance_m'] = round(lidar_distance, 3)
                obj['distance_source'] = 'lidar_projected'
            else:
                obj['distance_m'] = optional_float(obj.get('distance_estimate_m'))
                obj['distance_source'] = 'vision_estimate'
            prepared.append(obj)
        return prepared

    def distance_from_lidar(self, bbox: Any) -> float | None:
        if not isinstance(bbox, dict):
            return None
        x_min = optional_float(bbox.get('x_min'))
        x_max = optional_float(bbox.get('x_max'))
        if x_min is None or x_max is None:
            return None
        cx = clamp((x_min + x_max) * 0.5, 0.0, 1.0)
        width = max(0.05, clamp(abs(x_max - x_min), 0.0, 1.0))
        target_angle = (0.5 - cx) * self.camera_horizontal_fov
        half_window = max(math.radians(2.0), width * self.camera_horizontal_fov * 0.25)
        with self.scan_lock:
            scan = self.latest_scan
        if scan is None or scan.angle_increment == 0.0:
            return None
        values = []
        count = len(scan.ranges)
        for index, value in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment
            if abs(normalize_angle(angle - target_angle)) <= half_window:
                if math.isfinite(value) and scan.range_min <= value <= scan.range_max:
                    values.append(float(value))
        if not values:
            return None
        values.sort()
        percentile_index = max(0, min(len(values) - 1, int(len(values) * 0.35)))
        return values[percentile_index]

    def scan_summary(self) -> str:
        with self.scan_lock:
            scan = self.latest_scan
        if scan is None or not scan.ranges:
            return 'no lidar scan available'
        sectors = {
            'front': (math.radians(-12), math.radians(12)),
            'left': (math.radians(35), math.radians(75)),
            'right': (math.radians(-75), math.radians(-35)),
        }
        parts = []
        for name, (low, high) in sectors.items():
            values = []
            for index, value in enumerate(scan.ranges):
                angle = scan.angle_min + index * scan.angle_increment
                if low <= angle <= high and math.isfinite(value):
                    if scan.range_min <= value <= scan.range_max:
                        values.append(float(value))
            if values:
                values.sort()
                parts.append(f'{name}_median={values[len(values) // 2]:.2f}m')
        return ', '.join(parts) if parts else 'no finite lidar ranges in key sectors'

    def capture_frame(self, pose: dict[str, Any]) -> CapturedFrame | None:
        with self.image_lock:
            msg = self.latest_image
        if msg is None:
            return None
        rgb = self.image_to_rgb(msg)
        pil_image = PILImage.fromarray(rgb, mode='RGB')
        if pil_image.width > self.image_width:
            ratio = self.image_width / float(pil_image.width)
            height = max(1, int(pil_image.height * ratio))
            pil_image = pil_image.resize((self.image_width, height))
        buffer = io.BytesIO()
        pil_image.save(buffer, format='JPEG', quality=self.jpeg_quality, optimize=True)
        jpeg = buffer.getvalue()
        image_sha256 = hashlib.sha256(jpeg).hexdigest()
        image_hash = self.average_hash(pil_image)
        image_path = self.save_image(jpeg, image_sha256)
        return CapturedFrame(
            msg=msg,
            jpeg=jpeg,
            image_sha256=image_sha256,
            image_hash=image_hash,
            pose=pose,
            yaw=float(pose.get('yaw', 0.0)),
            image_path=image_path,
        )

    def save_image(self, jpeg: bytes, image_sha256: str) -> str | None:
        if self.image_dir is None:
            return None
        path = self.image_dir / f'{image_sha256[:16]}.jpg'
        if not path.exists():
            path.write_bytes(jpeg)
        return str(path)

    @staticmethod
    def average_hash(pil_image: Any) -> str:
        small = pil_image.convert('L').resize((8, 8))
        pixels = list(small.getdata())
        avg = sum(pixels) / len(pixels)
        bits = 0
        for value in pixels:
            bits = (bits << 1) | int(value >= avg)
        return f'{bits:016x}'

    @staticmethod
    def image_to_rgb(msg: Image) -> np.ndarray:
        encoding = msg.encoding.lower()
        channels_by_encoding = {
            'rgb8': 3,
            'bgr8': 3,
            'rgba8': 4,
            'bgra8': 4,
            'mono8': 1,
        }
        channels = channels_by_encoding.get(encoding)
        if channels is None:
            raise ValueError(f'unsupported image encoding: {msg.encoding}')
        expected_row_bytes = int(msg.width) * channels
        if int(msg.step) < expected_row_bytes:
            raise ValueError(f'image step {msg.step} is smaller than expected {expected_row_bytes}')
        raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        expected_bytes = int(msg.height) * int(msg.step)
        if raw.size < expected_bytes:
            raise ValueError(f'image data has {raw.size} bytes, expected at least {expected_bytes}')
        rows = raw[:expected_bytes].reshape((int(msg.height), int(msg.step)))
        active = rows[:, :expected_row_bytes].reshape((int(msg.height), int(msg.width), channels))
        if encoding == 'rgb8':
            return active[:, :, :3].copy()
        if encoding == 'bgr8':
            return active[:, :, [2, 1, 0]].copy()
        if encoding == 'rgba8':
            return active[:, :, :3].copy()
        if encoding == 'bgra8':
            return active[:, :, [2, 1, 0]].copy()
        mono = active[:, :, 0]
        return np.repeat(mono[:, :, None], 3, axis=2)

    def lookup_map_pose(self) -> dict[str, Any] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.args.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1),
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return {
            'frame_id': 'map',
            'position': {
                'x': float(translation.x),
                'y': float(translation.y),
                'z': float(translation.z),
            },
            'orientation': {
                'x': float(rotation.x),
                'y': float(rotation.y),
                'z': float(rotation.z),
                'w': float(rotation.w),
            },
            'yaw': yaw_from_quaternion(rotation),
        }

    @staticmethod
    def pose_from_request(payload: dict[str, Any]) -> dict[str, Any] | None:
        pose = payload.get('pose')
        if isinstance(pose, dict) and isinstance(pose.get('position'), dict):
            return pose
        target = payload.get('target')
        if isinstance(target, dict) and 'x' in target and 'y' in target:
            return {
                'frame_id': 'map',
                'position': {'x': float(target['x']), 'y': float(target['y']), 'z': 0.0},
                'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
                'yaw': 0.0,
            }
        return None

    def rotate_toward(self, yaw_error: float) -> None:
        twist = Twist()
        velocity = clamp(yaw_error * 1.4, -self.angular_speed, self.angular_speed)
        if abs(velocity) < 0.12:
            velocity = 0.12 if yaw_error > 0 else -0.12
        twist.angular.z = velocity
        self.cmd_vel_pub.publish(twist)

    def stop_robot(self) -> None:
        self.cmd_vel_pub.publish(Twist())

    def voltage_is_low(self) -> bool:
        return (
            self.min_voltage > 0.0
            and self.latest_voltage is not None
            and self.latest_voltage < self.min_voltage
        )

    def advance_to_next_view(self) -> None:
        if self.session is None:
            return
        self.session['index'] = int(self.session['index']) + 1
        self.session['state'] = 'rotating'
        self.session['settled_since'] = None

    def session_image_seen(self, image_hash: str) -> bool:
        if self.session is None:
            return False
        seen = self.session.get('seen_image_hashes')
        if not isinstance(seen, set):
            return False
        for previous in seen:
            if self.hamming_distance_hex(image_hash, str(previous)) <= self.max_similar_hamming:
                return True
        return False

    @staticmethod
    def hamming_distance_hex(left: str, right: str) -> int:
        try:
            a = int(left, 16)
            b = int(right, 16)
        except ValueError:
            return 999
        return int((a ^ b).bit_count())

    def finish_session(self) -> None:
        if self.session is None:
            return
        request_id = self.session['request_id']
        captures = list(self.session['captures'])
        objects = int(self.session['objects'])
        skipped_views = int(self.session['skipped_views'])
        self.stop_robot()
        self.session = None
        self.publish_capture_status(
            'completed',
            request_id=request_id,
            captures=captures,
            object_count=objects,
            skipped_views=skipped_views,
        )

    def publish_remember(self, payload: dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.remember_pub.publish(msg)

    def publish_capture_status(self, state: str, **extra: Any) -> None:
        payload = {
            'state': state,
            'stamp': self.get_clock().now().nanoseconds / 1e9,
            'source': CAPTURE_SOURCE,
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.get_logger().info(msg.data)
        self.capture_status_pub.publish(msg)
        self.status_pub.publish(msg)

    def publish_status(self, state: str, **extra: Any) -> None:
        payload = {
            'state': state,
            'stamp': self.get_clock().now().nanoseconds / 1e9,
            'source': CAPTURE_SOURCE,
            'model': self.vision_model,
            'capture_angles': self.capture_angles,
            'capture_radius': self.capture_radius,
            'has_api_key': bool(self.api_key),
            'store_image_only_captures': self.store_image_only_captures,
            'landmark_fallback': self.landmark_fallback,
            'landmarks': len(self.landmarks),
            'voltage': None if self.latest_voltage is None else round(self.latest_voltage, 2),
            'min_voltage': self.min_voltage,
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        noisy_states = {'idle', 'rotating', 'waiting_for_map_pose', 'waiting_for_image'}
        now = time.monotonic()
        last_logged = self.last_status_log_by_state.get(state, 0.0)
        if state not in noisy_states or now - last_logged >= 5.0:
            self.get_logger().info(msg.data)
            self.last_status_log_by_state[state] = now
        self.status_pub.publish(msg)

    def destroy_node(self) -> bool:
        if rclpy.ok():
            self.stop_robot()
        return super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser(description='OpenAI frontier-triggered semantic camera observer')
    parser.add_argument('--image-topic', default='/camera/image')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--use-sim-time', action='store_true')
    parser.add_argument('--scan-topic', default='/scan')
    parser.add_argument('--cmd-vel-topic', default='/cmd_vel')
    parser.add_argument('--voltage-topic', default='/voltage')
    parser.add_argument('--min-voltage', type=float, default=10.5,
                        help='Refuse or abort sweeps below this voltage; 0 disables the interlock')
    parser.add_argument('--remember-topic', default='/semantic_memory/remember')
    parser.add_argument('--status-topic', default='/semantic_observer/status')
    parser.add_argument('--capture-request-topic', default='/semantic_capture/request')
    parser.add_argument('--capture-status-topic', default='/semantic_capture/status')
    parser.add_argument('--vector-store', type=Path, default=default_vector_store_path())
    parser.add_argument(
        '--image-dir',
        default=str(Path(__file__).resolve().parent.parent / 'data' / 'semantic_images'),
    )
    parser.add_argument('--vision-model', default=DEFAULT_VISION_MODEL)
    parser.add_argument('--embedding-model', default=DEFAULT_EMBEDDING_MODEL)
    parser.add_argument('--api-key-env', default='OPENAI_API_KEY')
    parser.add_argument('--api-base', default=None)
    parser.add_argument('--openai-timeout', type=float, default=45.0)
    parser.add_argument('--capture-angles', type=int, default=4)
    parser.add_argument('--capture-radius', type=float, default=1.0)
    parser.add_argument('--area-captured-min-views', type=int, default=3)
    parser.add_argument('--min-confidence', type=float, default=0.35)
    parser.add_argument('--angular-speed', type=float, default=0.45)
    parser.add_argument('--yaw-tolerance', type=float, default=0.12)
    parser.add_argument('--settle-time', type=float, default=0.35)
    parser.add_argument('--capture-timeout', type=float, default=120.0)
    parser.add_argument('--camera-horizontal-fov', type=float, default=1.0472)
    parser.add_argument('--image-width', type=int, default=512)
    parser.add_argument('--jpeg-quality', type=int, default=72)
    parser.add_argument('--image-detail', default='low', choices=['low', 'high', 'auto'])
    parser.add_argument('--max-output-tokens', type=int, default=1800)
    parser.add_argument('--max-similar-hamming', type=int, default=6)
    parser.add_argument('--store-image-only-captures', default='true',
                        help='Save and remember frontier JPEG captures even when semantic analysis is unavailable')
    parser.add_argument('--landmarks', default='',
                        help='Optional JSON file of known world landmarks used as a local semantic fallback')
    parser.add_argument('--landmark-fallback', default='true',
                        help='Use the landmarks file when OpenAI vision is unavailable or fails')
    parser.add_argument('--landmark-max-distance', type=float, default=5.5,
                        help='Maximum distance for known-landmark fallback observations')
    parser.add_argument('--landmark-fov-margin', type=float, default=0.18,
                        help='Extra yaw margin outside the camera FOV for landmark fallback matching')
    parser.add_argument('--landmark-max-objects-per-view', type=int, default=6,
                        help='Maximum known landmarks to publish per captured view')
    args = parser.parse_args(remove_ros_args()[1:])

    rclpy.init()
    node = OpenAISemanticObserver(args)
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
