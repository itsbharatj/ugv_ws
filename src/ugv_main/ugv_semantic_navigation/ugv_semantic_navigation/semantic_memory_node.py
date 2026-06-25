#!/usr/bin/env python3
"""Semantic object memory for Example 10.

The node stores where the robot was in map coordinates when MCP/LLM reported
seeing an object. These are viewing poses, not estimated object centroids.
"""

import argparse
import json
import re
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from ugv_semantic_navigation.semantic_vector_store import (
    DEFAULT_EMBEDDING_MODEL,
    SemanticVectorStore,
    default_store_path as default_vector_store_path,
)


SCHEMA_VERSION = 1


def default_store_path() -> Path:
    return Path(__file__).resolve().parent.parent / 'data' / 'objects.json'


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def normalize_label(label: str) -> str:
    return re.sub(r'\s+', ' ', label.strip().lower())


def normalize_aliases(value: Any) -> list[str]:
    if value is None:
        return []
    raw_aliases: list[Any]
    if isinstance(value, list):
        raw_aliases = value
    else:
        raw_aliases = [value]

    aliases = []
    seen = set()
    for alias in raw_aliases:
        normalized = normalize_label(str(alias))
        if normalized and normalized not in seen:
            aliases.append(normalized)
            seen.add(normalized)
    return aliases


def yaw_from_quaternion(q: Any) -> float:
    import math

    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class SemanticMemoryNode(Node):
    """Append object observations to a JSON store and publish the full memory."""

    def __init__(self, store_path: Path, vector_store_path: Path, embedding_model: str,
                 base_frame: str = 'base_link', use_sim_time: bool = False):
        super().__init__(
            'semantic_memory_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)],
        )
        self.base_frame = base_frame
        self.store_path = store_path
        self.vector_store = SemanticVectorStore(vector_store_path, embedding_model=embedding_model)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        memory_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.objects_pub = self.create_publisher(
            String, '/semantic_memory/objects', memory_qos)
        self.status_pub = self.create_publisher(
            String, '/semantic_memory/status', 10)
        self.query_results_pub = self.create_publisher(
            String, '/semantic_memory/query_results', 10)
        self.remember_sub = self.create_subscription(
            String, '/semantic_memory/remember', self.remember_callback, 10)
        self.query_sub = self.create_subscription(
            String, '/semantic_memory/query', self.query_callback, 10)

        self.ensure_store()
        self.backfill_vector_store()
        self.publish_memory()
        self.get_logger().info(
            f'Semantic memory store: {self.store_path}; vector store: {vector_store_path}')

    def ensure_store(self) -> None:
        self.store_path.parent.mkdir(parents=True, exist_ok=True)
        if self.store_path.exists():
            return
        self.write_memory({'schema_version': SCHEMA_VERSION, 'objects': []})

    def read_memory(self) -> dict[str, Any]:
        with self.store_path.open('r', encoding='utf-8') as handle:
            data = json.load(handle)
        if not isinstance(data, dict):
            raise ValueError('semantic memory root must be an object')
        if data.get('schema_version') != SCHEMA_VERSION:
            data['schema_version'] = SCHEMA_VERSION
        if not isinstance(data.get('objects'), list):
            data['objects'] = []
        return data

    def write_memory(self, data: dict[str, Any]) -> None:
        tmp_path = self.store_path.with_suffix(self.store_path.suffix + '.tmp')
        with tmp_path.open('w', encoding='utf-8') as handle:
            json.dump(data, handle, indent=2, sort_keys=True)
            handle.write('\n')
        tmp_path.replace(self.store_path)

    def backfill_vector_store(self) -> None:
        try:
            memory = self.read_memory()
        except (OSError, ValueError, json.JSONDecodeError):
            return
        for observation in memory.get('objects', []):
            if not isinstance(observation, dict):
                continue
            observation_id = str(observation.get('id') or '')
            if not observation_id or self.vector_store.object_exists(observation_id):
                continue
            self.store_legacy_vector_observation(observation)

    def remember_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.publish_status('remember_error', error=f'invalid JSON: {exc}')
            return

        if payload.get('type') == 'semantic_capture':
            self.remember_capture(payload)
            return

        label = str(payload.get('label', '')).strip()
        label_normalized = normalize_label(label)
        if not label_normalized:
            self.publish_status('remember_error', error='missing label')
            return

        pose = self.lookup_map_pose()
        if pose is None:
            self.publish_status('remember_error', error='map->base_link transform unavailable')
            return

        try:
            memory = self.read_memory()
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.publish_status('remember_error', error=f'cannot read memory store: {exc}')
            return

        observation = {
            'id': f'obs_{uuid.uuid4().hex[:12]}',
            'schema_version': SCHEMA_VERSION,
            'timestamp': utc_now(),
            'label': label,
            'label_normalized': label_normalized,
            'aliases': normalize_aliases(payload.get('aliases')),
            'confidence': self.optional_float(payload.get('confidence')),
            'notes': str(payload.get('notes', '')).strip(),
            'source': str(payload.get('source', 'camera')).strip() or 'camera',
            'details': payload.get('details') if isinstance(payload.get('details'), dict) else {},
            'pose': pose,
        }

        memory['objects'].append(observation)
        try:
            self.write_memory(memory)
        except OSError as exc:
            self.publish_status('remember_error', error=f'cannot write memory store: {exc}')
            return

        self.store_legacy_vector_observation(observation)
        self.publish_memory(memory)
        self.publish_status(
            'remembered',
            id=observation['id'],
            label=label_normalized,
            pose=pose,
            object_count=len(memory['objects']),
        )

    def remember_capture(self, payload: dict[str, Any]) -> None:
        capture = payload.get('capture')
        if not isinstance(capture, dict):
            self.publish_status('remember_error', error='semantic_capture payload missing capture')
            return
        objects = payload.get('objects')
        if not isinstance(objects, list):
            self.publish_status('remember_error', error='semantic_capture payload missing objects array')
            return

        pose = capture.get('pose') if isinstance(capture.get('pose'), dict) else None
        if pose is None:
            pose = self.lookup_map_pose()
        if pose is None:
            self.publish_status('remember_error', error='map pose unavailable for semantic_capture')
            return
        capture['pose'] = pose

        try:
            memory = self.read_memory()
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            self.publish_status('remember_error', error=f'cannot read memory store: {exc}')
            return

        observations: list[dict[str, Any]] = []
        vector_objects: list[dict[str, Any]] = []
        for raw_obj in objects:
            if not isinstance(raw_obj, dict):
                continue
            label = str(raw_obj.get('label', '')).strip()
            label_normalized = normalize_label(label)
            if not label_normalized:
                continue
            observation_id = str(raw_obj.get('id') or f'obs_{uuid.uuid4().hex[:12]}')
            description = str(raw_obj.get('description') or raw_obj.get('notes') or '').strip()
            details = {
                'capture_id': capture.get('id'),
                'request_id': payload.get('request_id') or capture.get('request_id'),
                'description': description,
                'visibility': raw_obj.get('visibility'),
                'visible_fraction': raw_obj.get('visible_fraction'),
                'is_completely_visible': raw_obj.get('is_completely_visible'),
                'bbox': raw_obj.get('bbox') if isinstance(raw_obj.get('bbox'), dict) else {},
                'distance_m': raw_obj.get('distance_m'),
                'distance_source': raw_obj.get('distance_source'),
                'image_sha256': capture.get('image_sha256'),
                'image_hash': capture.get('image_hash'),
                'image_path': capture.get('image_path'),
                'model': capture.get('model'),
                'scene_summary': capture.get('summary'),
            }
            observation = {
                'id': observation_id,
                'schema_version': SCHEMA_VERSION,
                'timestamp': str(capture.get('timestamp') or utc_now()),
                'label': label,
                'label_normalized': label_normalized,
                'aliases': normalize_aliases(raw_obj.get('aliases')),
                'confidence': self.optional_float(raw_obj.get('confidence')),
                'notes': description,
                'source': str(raw_obj.get('source') or capture.get('source') or 'openai_vla'),
                'details': details,
                'pose': pose,
            }
            observations.append(observation)
            vector_obj = dict(raw_obj)
            vector_obj['id'] = observation_id
            vector_obj['source'] = observation['source']
            vector_objects.append(vector_obj)

        memory['objects'].extend(observations)
        try:
            self.write_memory(memory)
        except OSError as exc:
            self.publish_status('remember_error', error=f'cannot write memory store: {exc}')
            return

        try:
            self.vector_store.add_capture(capture, vector_objects, embed=True)
        except Exception as exc:  # noqa: BLE001 - vector persistence should not kill JSON memory.
            self.publish_status('vector_store_error', error=str(exc), capture_id=capture.get('id'))

        self.publish_memory(memory)
        self.publish_status(
            'capture_remembered',
            capture_id=capture.get('id'),
            request_id=payload.get('request_id') or capture.get('request_id'),
            added=len(observations),
            object_count=len(memory['objects']),
            vector_store=str(self.vector_store.db_path),
        )

    def store_legacy_vector_observation(self, observation: dict[str, Any]) -> None:
        capture = {
            'id': f"cap_{observation['id']}",
            'request_id': None,
            'timestamp': observation.get('timestamp'),
            'source': observation.get('source', 'camera'),
            'pose': observation.get('pose'),
            'summary': observation.get('notes', ''),
            'model': None,
            'raw_response': {},
        }
        obj = {
            'id': observation['id'],
            'label': observation.get('label'),
            'aliases': observation.get('aliases', []),
            'description': observation.get('notes', ''),
            'confidence': observation.get('confidence'),
            'visibility': observation.get('details', {}).get('visibility', 'uncertain'),
            'visible_fraction': observation.get('details', {}).get('visible_fraction'),
            'is_completely_visible': observation.get('details', {}).get('is_completely_visible', False),
            'bbox': observation.get('details', {}).get('bbox', {}),
            'distance_m': observation.get('details', {}).get('distance_m'),
            'distance_source': observation.get('details', {}).get('distance_source'),
            'source': observation.get('source', 'camera'),
        }
        try:
            self.vector_store.add_capture(capture, [obj], embed=True)
        except Exception as exc:  # noqa: BLE001
            self.publish_status('vector_store_error', error=str(exc), observation_id=observation['id'])

    def query_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.publish_query_results('', [], error=f'invalid JSON: {exc}')
            return

        query = str(payload.get('query') or payload.get('label') or '').strip()
        label = str(payload.get('label') or '').strip() or None
        if not query:
            self.publish_query_results('', [], error='missing query or label')
            return
        try:
            top_k = int(payload.get('top_k', 5))
        except (TypeError, ValueError):
            top_k = 5
        include_duplicates = bool(payload.get('include_duplicates', False))
        try:
            per_label_limit = int(payload.get('per_label_limit', 2))
        except (TypeError, ValueError):
            per_label_limit = 2
        try:
            diversity_radius = float(payload.get('diversity_radius', 0.75))
        except (TypeError, ValueError):
            diversity_radius = 0.75

        results = self.vector_store.search(
            query,
            top_k=top_k,
            label=label,
            include_duplicates=include_duplicates,
            per_label_limit=per_label_limit,
            diversity_radius=diversity_radius,
        )
        self.publish_query_results(query, results, request_id=payload.get('request_id'))

    def lookup_map_pose(self) -> dict[str, Any] | None:
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
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

    def publish_memory(self, memory: dict[str, Any] | None = None) -> None:
        if memory is None:
            try:
                memory = self.read_memory()
            except (OSError, ValueError, json.JSONDecodeError) as exc:
                self.publish_status('memory_error', error=f'cannot read memory store: {exc}')
                return
        msg = String()
        msg.data = json.dumps(memory, sort_keys=True)
        self.objects_pub.publish(msg)

    def publish_status(self, state: str, **extra: Any) -> None:
        payload = {
            'state': state,
            'stamp': self.get_clock().now().nanoseconds / 1e9,
            'store': str(self.store_path),
            'vector_store': str(self.vector_store.db_path),
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.status_pub.publish(msg)

    def publish_query_results(
        self,
        query: str,
        results: list[dict[str, Any]],
        **extra: Any,
    ) -> None:
        payload = {
            'type': 'semantic_query_results',
            'query': query,
            'count': len(results),
            'results': results,
            'stamp': self.get_clock().now().nanoseconds / 1e9,
            'vector_store': str(self.vector_store.db_path),
        }
        payload.update(extra)
        msg = String()
        msg.data = json.dumps(payload, sort_keys=True)
        self.query_results_pub.publish(msg)
        self.publish_status('query_completed', query=query, result_count=len(results))

    @staticmethod
    def optional_float(value: Any) -> float | None:
        if value is None or value == '':
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None


def main() -> None:
    parser = argparse.ArgumentParser(description='Semantic object memory node')
    parser.add_argument('--store', type=Path, default=default_store_path(),
                        help='Path to the semantic object memory JSON file')
    parser.add_argument('--vector-store', type=Path, default=default_vector_store_path(),
                        help='Path to semantic vector SQLite database')
    parser.add_argument('--embedding-model', default=DEFAULT_EMBEDDING_MODEL,
                        help='OpenAI embedding model for semantic retrieval')
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--use-sim-time', action='store_true')
    args = parser.parse_args(remove_ros_args()[1:])

    rclpy.init()
    node = SemanticMemoryNode(
        args.store, args.vector_store, args.embedding_model, args.base_frame, args.use_sim_time)
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
