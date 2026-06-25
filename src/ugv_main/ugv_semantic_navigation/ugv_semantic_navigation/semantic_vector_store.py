#!/usr/bin/env python3
"""SQLite-backed semantic memory with optional OpenAI embeddings.

The store is intentionally service-free: it gives this demo vector retrieval
without requiring Chroma, Qdrant, or a database daemon. Embeddings are stored as
float32 blobs and ranked in process with cosine similarity.
"""

from __future__ import annotations

import json
import math
import os
import re
import sqlite3
import struct
import urllib.error
import urllib.request
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


SCHEMA_VERSION = 2
DEFAULT_EMBEDDING_MODEL = 'text-embedding-3-small'
DEFAULT_CAMERA_HFOV = 1.0472  # ~60 deg


def estimate_object_position(
    robot_x: float | None,
    robot_y: float | None,
    robot_yaw: float | None,
    bbox: Any,
    distance_m: float | None,
    fov: float = DEFAULT_CAMERA_HFOV,
) -> tuple[float | None, float | None, float | None]:
    if (
        robot_x is None or robot_y is None or robot_yaw is None
        or distance_m is None or not isinstance(bbox, dict)
    ):
        return None, None, None
    x_min = optional_float(bbox.get('x_min'))
    x_max = optional_float(bbox.get('x_max'))
    if x_min is None or x_max is None:
        return None, None, None
    cx = max(0.0, min(1.0, (float(x_min) + float(x_max)) * 0.5))
    bearing = (0.5 - cx) * float(fov)
    obj_x = float(robot_x) + float(distance_m) * math.cos(float(robot_yaw) + bearing)
    obj_y = float(robot_y) + float(distance_m) * math.sin(float(robot_yaw) + bearing)
    return bearing, obj_x, obj_y


def default_store_path() -> Path:
    return Path.home() / '.ros' / 'ugv_semantic_navigation' / 'semantic_memory.sqlite3'


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec='milliseconds').replace('+00:00', 'Z')


def normalize_label(label: str) -> str:
    return re.sub(r'\s+', ' ', label.strip().lower())


def tokenize(text: str) -> set[str]:
    return {token for token in re.split(r'[^a-z0-9]+', text.lower()) if token}


def optional_float(value: Any) -> float | None:
    if value is None or value == '':
        return None
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(number):
        return None
    return number


def vector_to_blob(vector: list[float]) -> bytes:
    if not vector:
        return b''
    return struct.pack('<%sf' % len(vector), *[float(value) for value in vector])


def blob_to_vector(blob: bytes | None) -> list[float]:
    if not blob:
        return []
    count = len(blob) // 4
    return list(struct.unpack('<%sf' % count, blob))


def cosine_similarity(left: list[float], right: list[float]) -> float:
    if not left or not right or len(left) != len(right):
        return 0.0
    dot = 0.0
    left_norm = 0.0
    right_norm = 0.0
    for a, b in zip(left, right):
        dot += a * b
        left_norm += a * a
        right_norm += b * b
    if left_norm <= 0.0 or right_norm <= 0.0:
        return 0.0
    return dot / math.sqrt(left_norm * right_norm)


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def visibility_score(visibility: str | None, visible_fraction: float | None) -> float:
    visibility_map = {
        'full': 1.0,
        'mostly_visible': 0.85,
        'partial': 0.55,
        'occluded': 0.35,
        'uncertain': 0.45,
    }
    score = visibility_map.get((visibility or '').strip().lower(), 0.5)
    if visible_fraction is not None:
        score = 0.5 * score + 0.5 * max(0.0, min(1.0, visible_fraction))
    return score


def pose_distance(a: dict[str, Any], b: dict[str, Any]) -> float:
    ax = optional_float(a.get('x'))
    ay = optional_float(a.get('y'))
    bx = optional_float(b.get('x'))
    by = optional_float(b.get('y'))
    if ax is None or ay is None or bx is None or by is None:
        return float('inf')
    return math.hypot(ax - bx, ay - by)


class SemanticVectorStore:
    """Persist captures and object observations, then retrieve ranked views."""

    def __init__(
        self,
        db_path: Path | str | None = None,
        embedding_model: str = DEFAULT_EMBEDDING_MODEL,
        api_key: str | None = None,
        api_base: str | None = None,
        request_timeout: float = 30.0,
        camera_horizontal_fov: float = DEFAULT_CAMERA_HFOV,
    ):
        self.db_path = Path(db_path) if db_path is not None else default_store_path()
        self.embedding_model = embedding_model
        self.camera_horizontal_fov = float(camera_horizontal_fov)
        self.api_key = api_key if api_key is not None else os.environ.get('OPENAI_API_KEY')
        self.api_base = (api_base or os.environ.get('OPENAI_BASE_URL') or 'https://api.openai.com/v1').rstrip('/')
        self.request_timeout = max(1.0, float(request_timeout))
        self.db_path.parent.mkdir(parents=True, exist_ok=True)
        self.ensure_schema()

    def ensure_schema(self) -> None:
        with sqlite3.connect(self.db_path) as conn:
            conn.execute('PRAGMA journal_mode=WAL')
            conn.execute('PRAGMA foreign_keys=ON')
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS captures (
                    id TEXT PRIMARY KEY,
                    request_id TEXT,
                    area_key TEXT,
                    timestamp TEXT NOT NULL,
                    source TEXT NOT NULL,
                    x REAL,
                    y REAL,
                    yaw REAL,
                    image_sha256 TEXT,
                    image_hash TEXT,
                    image_path TEXT,
                    image_stamp_sec INTEGER,
                    image_stamp_nanosec INTEGER,
                    summary TEXT,
                    model TEXT,
                    raw_json TEXT,
                    schema_version INTEGER NOT NULL
                )
                """
            )
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS objects (
                    id TEXT PRIMARY KEY,
                    capture_id TEXT NOT NULL REFERENCES captures(id) ON DELETE CASCADE,
                    timestamp TEXT NOT NULL,
                    source TEXT NOT NULL,
                    label TEXT NOT NULL,
                    label_normalized TEXT NOT NULL,
                    aliases_json TEXT NOT NULL,
                    description TEXT NOT NULL,
                    confidence REAL,
                    visibility TEXT,
                    visible_fraction REAL,
                    is_completely_visible INTEGER,
                    distance_m REAL,
                    distance_source TEXT,
                    bbox_json TEXT,
                    x REAL,
                    y REAL,
                    yaw REAL,
                    pose_json TEXT NOT NULL,
                    navigation_text TEXT NOT NULL,
                    raw_json TEXT,
                    schema_version INTEGER NOT NULL
                )
                """
            )
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS object_embeddings (
                    object_id TEXT PRIMARY KEY REFERENCES objects(id) ON DELETE CASCADE,
                    model TEXT NOT NULL,
                    dimensions INTEGER NOT NULL,
                    vector BLOB NOT NULL
                )
                """
            )
            conn.execute('CREATE INDEX IF NOT EXISTS idx_captures_xy ON captures(x, y)')
            conn.execute('CREATE INDEX IF NOT EXISTS idx_objects_label ON objects(label_normalized)')
            conn.execute('CREATE INDEX IF NOT EXISTS idx_objects_xy ON objects(x, y)')
            for col_def in (
                'bearing_rad REAL',
                'estimated_object_x REAL',
                'estimated_object_y REAL',
            ):
                try:
                    conn.execute(f'ALTER TABLE objects ADD COLUMN {col_def}')
                except sqlite3.OperationalError:
                    pass
            conn.execute(
                'CREATE INDEX IF NOT EXISTS idx_objects_est_xy '
                'ON objects(estimated_object_x, estimated_object_y)'
            )

    def add_capture(
        self,
        capture: dict[str, Any],
        objects: list[dict[str, Any]],
        *,
        embed: bool = True,
    ) -> list[dict[str, Any]]:
        """Insert one image capture and its object observations."""
        capture_id = str(capture.get('id') or f'cap_{uuid.uuid4().hex[:12]}')
        timestamp = str(capture.get('timestamp') or utc_now())
        pose = capture.get('pose') if isinstance(capture.get('pose'), dict) else {}
        position = pose.get('position') if isinstance(pose.get('position'), dict) else {}
        x = optional_float(capture.get('x'))
        y = optional_float(capture.get('y'))
        yaw = optional_float(capture.get('yaw'))
        if x is None:
            x = optional_float(position.get('x'))
        if y is None:
            y = optional_float(position.get('y'))
        if yaw is None:
            yaw = optional_float(pose.get('yaw'))

        scene_summary = str(capture.get('summary') or '').strip()
        normalized_objects = [
            self._normalize_object(obj, capture_id, timestamp, pose, x, y, yaw, scene_summary)
            for obj in objects
            if normalize_label(str(obj.get('label', '')))
        ]

        embedding_texts = [obj['navigation_text'] for obj in normalized_objects]
        embeddings = self.embed_texts(embedding_texts) if embed and embedding_texts else []
        if len(embeddings) != len(normalized_objects):
            embeddings = []

        with sqlite3.connect(self.db_path) as conn:
            conn.execute('PRAGMA foreign_keys=ON')
            conn.execute(
                """
                INSERT OR REPLACE INTO captures (
                    id, request_id, area_key, timestamp, source, x, y, yaw,
                    image_sha256, image_hash, image_path, image_stamp_sec,
                    image_stamp_nanosec, summary, model, raw_json, schema_version
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    capture_id,
                    capture.get('request_id'),
                    capture.get('area_key'),
                    timestamp,
                    str(capture.get('source') or 'openai_vla'),
                    x,
                    y,
                    yaw,
                    capture.get('image_sha256'),
                    capture.get('image_hash'),
                    capture.get('image_path'),
                    capture.get('image_stamp_sec'),
                    capture.get('image_stamp_nanosec'),
                    scene_summary,
                    capture.get('model'),
                    json.dumps(capture.get('raw_response', {}), sort_keys=True),
                    SCHEMA_VERSION,
                ),
            )
            for index, obj in enumerate(normalized_objects):
                conn.execute(
                    """
                    INSERT OR REPLACE INTO objects (
                        id, capture_id, timestamp, source, label, label_normalized,
                        aliases_json, description, confidence, visibility, visible_fraction,
                        is_completely_visible, distance_m, distance_source, bbox_json,
                        x, y, yaw, pose_json, navigation_text, raw_json, schema_version,
                        bearing_rad, estimated_object_x, estimated_object_y
                    ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
                    """,
                    (
                        obj['id'],
                        capture_id,
                        timestamp,
                        obj['source'],
                        obj['label'],
                        obj['label_normalized'],
                        json.dumps(obj['aliases'], sort_keys=True),
                        obj['description'],
                        obj['confidence'],
                        obj['visibility'],
                        obj['visible_fraction'],
                        1 if obj['is_completely_visible'] else 0,
                        obj['distance_m'],
                        obj['distance_source'],
                        json.dumps(obj['bbox'], sort_keys=True),
                        x,
                        y,
                        yaw,
                        json.dumps(pose, sort_keys=True),
                        obj['navigation_text'],
                        json.dumps(obj['raw'], sort_keys=True),
                        SCHEMA_VERSION,
                        obj.get('bearing_rad'),
                        obj.get('estimated_object_x'),
                        obj.get('estimated_object_y'),
                    ),
                )
                if embeddings:
                    vector = embeddings[index]
                    conn.execute(
                        """
                        INSERT OR REPLACE INTO object_embeddings (
                            object_id, model, dimensions, vector
                        ) VALUES (?, ?, ?, ?)
                        """,
                        (obj['id'], self.embedding_model, len(vector), vector_to_blob(vector)),
                    )
        return normalized_objects

    def _normalize_object(
        self,
        obj: dict[str, Any],
        capture_id: str,
        timestamp: str,
        pose: dict[str, Any],
        x: float | None,
        y: float | None,
        yaw: float | None,
        scene_summary: str,
    ) -> dict[str, Any]:
        label = str(obj.get('label') or '').strip()
        label_normalized = normalize_label(label)
        aliases_value = obj.get('aliases')
        if isinstance(aliases_value, list):
            aliases = [normalize_label(str(alias)) for alias in aliases_value]
        elif aliases_value:
            aliases = [normalize_label(str(aliases_value))]
        else:
            aliases = []
        aliases = [alias for alias in dict.fromkeys(aliases) if alias and alias != label_normalized]
        description = str(obj.get('description') or obj.get('notes') or label).strip()
        confidence = optional_float(obj.get('confidence'))
        visible_fraction = optional_float(obj.get('visible_fraction'))
        visibility = str(obj.get('visibility') or 'uncertain').strip().lower()
        is_completely_visible = bool(
            obj.get('is_completely_visible')
            if 'is_completely_visible' in obj
            else visibility in ('full', 'mostly_visible') and (visible_fraction or 0.0) >= 0.85
        )
        distance_m = optional_float(obj.get('distance_m'))
        if distance_m is None:
            distance_m = optional_float(obj.get('distance_estimate_m'))
        distance_source = str(obj.get('distance_source') or 'vision_estimate').strip()
        bbox = obj.get('bbox') if isinstance(obj.get('bbox'), dict) else {}

        bearing_rad, est_x, est_y = estimate_object_position(
            x, y, yaw, bbox, distance_m, self.camera_horizontal_fov,
        )

        pose_bits = []
        if x is not None and y is not None:
            pose_bits.append(f'map position x={x:.2f} y={y:.2f}')
        if yaw is not None:
            pose_bits.append(f'camera yaw={yaw:.2f} rad')
        if distance_m is not None:
            pose_bits.append(f'object distance about {distance_m:.2f} meters')
        pose_text = '; '.join(pose_bits)

        navigation_text = (
            f'{label}. {description}. '
            f'Aliases: {", ".join(aliases) if aliases else "none"}. '
            f'Visibility: {visibility}, visible_fraction={visible_fraction}. '
            f'Confidence: {confidence}. {pose_text}. Scene: {scene_summary}'
        )

        return {
            'id': str(obj.get('id') or f'obs_{uuid.uuid4().hex[:12]}'),
            'capture_id': capture_id,
            'timestamp': timestamp,
            'source': str(obj.get('source') or 'openai_vla'),
            'label': label,
            'label_normalized': label_normalized,
            'aliases': aliases,
            'description': description,
            'confidence': confidence,
            'visibility': visibility,
            'visible_fraction': visible_fraction,
            'is_completely_visible': is_completely_visible,
            'distance_m': distance_m,
            'distance_source': distance_source,
            'bbox': bbox,
            'pose': pose,
            'navigation_text': navigation_text,
            'bearing_rad': bearing_rad,
            'estimated_object_x': est_x,
            'estimated_object_y': est_y,
            'raw': obj,
        }

    def embed_texts(self, texts: list[str]) -> list[list[float]]:
        if not texts or not self.api_key:
            return []
        try:
            response = self.openai_post('/embeddings', {
                'model': self.embedding_model,
                'input': texts,
            })
            by_index = sorted(response.get('data', []), key=lambda item: int(item.get('index', 0)))
            return [list(item.get('embedding', [])) for item in by_index]
        except Exception:
            return []

    def openai_post(self, path: str, payload: dict[str, Any]) -> dict[str, Any]:
        if not self.api_key:
            raise RuntimeError('OPENAI_API_KEY is not set')
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
            with urllib.request.urlopen(request, timeout=self.request_timeout) as response:
                return json.loads(response.read().decode('utf-8'))
        except urllib.error.HTTPError as exc:
            detail = exc.read().decode('utf-8', errors='replace')
            raise RuntimeError(f'OpenAI HTTP {exc.code}: {detail}') from exc

    def area_view_bucket_count(self, x: float, y: float, radius: float, buckets: int) -> int:
        buckets = max(1, int(buckets))
        rows = self._captures_near(x, y, radius)
        yaw_buckets = set()
        for row in rows:
            yaw = optional_float(row.get('yaw'))
            if yaw is None:
                continue
            bucket = int((((yaw % (2.0 * math.pi)) / (2.0 * math.pi)) * buckets))
            yaw_buckets.add(max(0, min(buckets - 1, bucket)))
        return len(yaw_buckets)

    def area_has_min_views(self, x: float, y: float, radius: float, min_views: int) -> bool:
        rows = self._captures_near(x, y, radius)
        if len(rows) < min_views:
            return False
        if min_views <= 1:
            return True
        return self.area_view_bucket_count(x, y, radius, min_views) >= min_views

    def similar_image_seen(
        self,
        image_hash: str,
        x: float | None,
        y: float | None,
        radius: float,
        max_hamming: int = 6,
    ) -> bool:
        if not image_hash or x is None or y is None:
            return False
        for row in self._captures_near(x, y, radius):
            previous = str(row.get('image_hash') or '')
            if previous and hamming_distance_hex(image_hash, previous) <= max_hamming:
                return True
        return False

    def _captures_near(self, x: float, y: float, radius: float) -> list[dict[str, Any]]:
        with sqlite3.connect(self.db_path) as conn:
            conn.row_factory = sqlite3.Row
            rows = conn.execute(
                """
                SELECT id, x, y, yaw, image_hash, timestamp
                FROM captures
                WHERE x BETWEEN ? AND ? AND y BETWEEN ? AND ?
                """,
                (x - radius, x + radius, y - radius, y + radius),
            ).fetchall()
        result = []
        for row in rows:
            item = dict(row)
            rx = optional_float(item.get('x'))
            ry = optional_float(item.get('y'))
            if rx is not None and ry is not None and math.hypot(rx - x, ry - y) <= radius:
                result.append(item)
        return result

    def search(
        self,
        query: str,
        *,
        top_k: int = 5,
        label: str | None = None,
        include_duplicates: bool = False,
        per_label_limit: int = 2,
        diversity_radius: float = 0.75,
    ) -> list[dict[str, Any]]:
        top_k = max(1, min(int(top_k), 25))
        label_normalized = normalize_label(label or '')
        query_text = query.strip()
        query_embedding = []
        if query_text and self.api_key:
            query_embeddings = self.embed_texts([query_text])
            if query_embeddings:
                query_embedding = query_embeddings[0]

        rows = self._load_object_rows(label_normalized if label_normalized else None)
        ranked = []
        for row in rows:
            vector = blob_to_vector(row.pop('vector', None))
            vector_score = cosine_similarity(query_embedding, vector) if query_embedding and vector else 0.0
            lexical = self._lexical_score(query_text, row)
            semantic_score = vector_score if vector_score > 0.0 else lexical
            exact_label_boost = self._label_boost(label_normalized or query_text, row)
            confidence = optional_float(row.get('confidence')) or 0.5
            visible_fraction = optional_float(row.get('visible_fraction'))
            visibility = visibility_score(row.get('visibility'), visible_fraction)
            distance_m = optional_float(row.get('distance_m'))
            distance_bonus = 0.0
            if distance_m is not None:
                distance_bonus = max(0.0, min(0.12, (5.0 - min(distance_m, 5.0)) / 5.0 * 0.12))
            final_score = (
                semantic_score
                + exact_label_boost
                + 0.16 * max(0.0, min(1.0, confidence))
                + 0.14 * visibility
                + distance_bonus
            )
            row['score'] = round(float(final_score), 4)
            row['similarity'] = round(float(vector_score), 4)
            row['lexical_score'] = round(float(lexical), 4)
            ranked.append(row)

        ranked.sort(key=lambda item: item['score'], reverse=True)
        if include_duplicates:
            return [self._public_object(row) for row in ranked[:top_k]]

        selected: list[dict[str, Any]] = []
        label_counts: dict[str, int] = {}
        for row in ranked:
            row_label = row['label_normalized']
            if label_counts.get(row_label, 0) >= per_label_limit:
                continue
            if self._too_similar_to_selected(row, selected, diversity_radius):
                continue
            selected.append(row)
            label_counts[row_label] = label_counts.get(row_label, 0) + 1
            if len(selected) >= top_k:
                break
        return [self._public_object(row) for row in selected]

    def _load_object_rows(self, label_normalized: str | None = None) -> list[dict[str, Any]]:
        sql = (
            """
            SELECT
                objects.*,
                object_embeddings.vector AS vector
            FROM objects
            LEFT JOIN object_embeddings ON object_embeddings.object_id = objects.id
            """
        )
        params: tuple[Any, ...] = ()
        if label_normalized:
            sql += ' WHERE objects.label_normalized = ? OR objects.aliases_json LIKE ?'
            params = (label_normalized, f'%"{label_normalized}"%')
        with sqlite3.connect(self.db_path) as conn:
            conn.row_factory = sqlite3.Row
            return [dict(row) for row in conn.execute(sql, params).fetchall()]

    def _lexical_score(self, query: str, row: dict[str, Any]) -> float:
        query_terms = tokenize(query)
        text = ' '.join([
            str(row.get('label') or ''),
            str(row.get('aliases_json') or ''),
            str(row.get('description') or ''),
            str(row.get('navigation_text') or ''),
        ])
        text_terms = tokenize(text)
        if not query_terms or not text_terms:
            return 0.0
        overlap = len(query_terms & text_terms)
        return overlap / math.sqrt(len(query_terms) * len(text_terms))

    def _label_boost(self, query_or_label: str, row: dict[str, Any]) -> float:
        normalized = normalize_label(query_or_label)
        if not normalized:
            return 0.0
        aliases = json.loads(row.get('aliases_json') or '[]')
        candidates = {row.get('label_normalized'), normalize_label(row.get('label') or '')}
        candidates.update(str(alias) for alias in aliases)
        if normalized in candidates:
            return 0.35
        if any(normalized in candidate or candidate in normalized for candidate in candidates if candidate):
            return 0.18
        return 0.0

    def _too_similar_to_selected(
        self,
        row: dict[str, Any],
        selected: list[dict[str, Any]],
        diversity_radius: float,
    ) -> bool:
        row_pose = {'x': row.get('x'), 'y': row.get('y')}
        row_yaw = optional_float(row.get('yaw'))
        for existing in selected:
            if existing.get('label_normalized') != row.get('label_normalized'):
                continue
            distance = pose_distance(row_pose, {'x': existing.get('x'), 'y': existing.get('y')})
            existing_yaw = optional_float(existing.get('yaw'))
            yaw_close = (
                row_yaw is not None
                and existing_yaw is not None
                and abs(normalize_angle(row_yaw - existing_yaw)) < 0.7
            )
            if distance < diversity_radius and (yaw_close or distance < diversity_radius * 0.5):
                return True
        return False

    def _public_object(self, row: dict[str, Any]) -> dict[str, Any]:
        pose = json.loads(row.get('pose_json') or '{}')
        bbox = json.loads(row.get('bbox_json') or '{}')
        aliases = json.loads(row.get('aliases_json') or '[]')
        return {
            'id': row.get('id'),
            'capture_id': row.get('capture_id'),
            'label': row.get('label'),
            'label_normalized': row.get('label_normalized'),
            'aliases': aliases,
            'description': row.get('description'),
            'confidence': row.get('confidence'),
            'visibility': row.get('visibility'),
            'visible_fraction': row.get('visible_fraction'),
            'is_completely_visible': bool(row.get('is_completely_visible')),
            'distance_m': row.get('distance_m'),
            'distance_source': row.get('distance_source'),
            'bbox': bbox,
            'pose': pose,
            'angle_yaw': row.get('yaw'),
            'bearing_rad': row.get('bearing_rad'),
            'estimated_object_x': row.get('estimated_object_x'),
            'estimated_object_y': row.get('estimated_object_y'),
            'score': row.get('score'),
            'similarity': row.get('similarity'),
            'source': row.get('source'),
            'timestamp': row.get('timestamp'),
            'navigation_text': row.get('navigation_text'),
        }

    def get_object(self, object_id: str) -> dict[str, Any] | None:
        rows = [row for row in self._load_object_rows() if row.get('id') == object_id]
        if not rows:
            return None
        return self._public_object(rows[0])

    def object_exists(self, object_id: str) -> bool:
        with sqlite3.connect(self.db_path) as conn:
            row = conn.execute('SELECT 1 FROM objects WHERE id = ? LIMIT 1', (object_id,)).fetchone()
        return row is not None

    def get_fused_objects(
        self,
        label: str | None = None,
        *,
        cluster_radius: float = 1.0,
        min_confidence: float = 0.0,
        exclude_sources: tuple[str, ...] = (),
    ) -> list[dict[str, Any]]:
        """Cluster observations by label+estimated position; return fused objects.

        Returns one entry per cluster with object_x, object_y, best observation,
        and all contributing observation rows. Observations missing an estimated
        position are skipped (they cannot drive standoff planning).
        """
        label_normalized = normalize_label(label or '') or None
        rows = self._load_object_rows(label_normalized)
        if exclude_sources:
            rows = [r for r in rows if str(r.get('source') or '') not in exclude_sources]
        if min_confidence > 0:
            rows = [r for r in rows if (optional_float(r.get('confidence')) or 0.0) >= min_confidence]

        usable = []
        for row in rows:
            ex = optional_float(row.get('estimated_object_x'))
            ey = optional_float(row.get('estimated_object_y'))
            if ex is None or ey is None:
                continue
            row['_ex'] = ex
            row['_ey'] = ey
            usable.append(row)

        def _row_score(row: dict[str, Any]) -> float:
            conf = optional_float(row.get('confidence')) or 0.5
            vis = visibility_score(row.get('visibility'), optional_float(row.get('visible_fraction')))
            lidar_bonus = 0.15 if str(row.get('distance_source') or '') == 'lidar_projected' else 0.0
            dist = optional_float(row.get('distance_m')) or 5.0
            close_bonus = max(0.0, (5.0 - min(dist, 5.0)) / 5.0) * 0.10
            return conf * 0.6 + vis * 0.3 + lidar_bonus + close_bonus

        usable.sort(key=_row_score, reverse=True)

        clusters: list[dict[str, Any]] = []
        for row in usable:
            row_label = str(row.get('label_normalized') or '')
            placed = False
            for cluster in clusters:
                if cluster['label_normalized'] != row_label:
                    continue
                if math.hypot(row['_ex'] - cluster['object_x'], row['_ey'] - cluster['object_y']) <= cluster_radius:
                    cluster['observations'].append(row)
                    n = len(cluster['observations'])
                    weights = [_row_score(o) for o in cluster['observations']]
                    total = sum(weights) or 1.0
                    cluster['object_x'] = sum(o['_ex'] * w for o, w in zip(cluster['observations'], weights)) / total
                    cluster['object_y'] = sum(o['_ey'] * w for o, w in zip(cluster['observations'], weights)) / total
                    cluster['n_observations'] = n
                    if _row_score(row) > _row_score(cluster['best_observation']):
                        cluster['best_observation'] = row
                    placed = True
                    break
            if not placed:
                aliases = []
                try:
                    aliases = json.loads(row.get('aliases_json') or '[]')
                except (TypeError, ValueError):
                    aliases = []
                clusters.append({
                    'object_x': row['_ex'],
                    'object_y': row['_ey'],
                    'label': row.get('label'),
                    'label_normalized': row_label,
                    'aliases': aliases,
                    'observations': [row],
                    'best_observation': row,
                    'n_observations': 1,
                })

        results: list[dict[str, Any]] = []
        for cluster in clusters:
            best = cluster['best_observation']
            results.append({
                'object_x': round(float(cluster['object_x']), 3),
                'object_y': round(float(cluster['object_y']), 3),
                'label': cluster['label'],
                'label_normalized': cluster['label_normalized'],
                'aliases': cluster['aliases'],
                'n_observations': cluster['n_observations'],
                'cluster_score': round(sum(_row_score(o) for o in cluster['observations']), 3),
                'best_observation': self._public_object(best),
                'observation_ids': [str(o.get('id')) for o in cluster['observations']],
                'sources': sorted({str(o.get('source') or '') for o in cluster['observations']}),
            })
        results.sort(key=lambda c: (c['n_observations'], c['cluster_score']), reverse=True)
        return results

    def list_labels(self) -> list[dict[str, Any]]:
        with sqlite3.connect(self.db_path) as conn:
            conn.row_factory = sqlite3.Row
            rows = conn.execute(
                """
                SELECT label_normalized AS label, COUNT(*) AS count,
                       AVG(confidence) AS avg_confidence
                FROM objects
                GROUP BY label_normalized
                ORDER BY count DESC, label ASC
                """
            ).fetchall()
        return [dict(row) for row in rows]


def hamming_distance_hex(left: str, right: str) -> int:
    try:
        a = int(left, 16)
        b = int(right, 16)
    except ValueError:
        return 999
    return int((a ^ b).bit_count())
