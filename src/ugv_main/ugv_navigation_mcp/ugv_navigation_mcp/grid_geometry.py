from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
from typing import Any, Iterable

import numpy as np


UNKNOWN = -1
DEFAULT_FREE_THRESHOLD = 20
DEFAULT_OCCUPIED_THRESHOLD = 65


@dataclass(frozen=True)
class GridInfo:
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    origin_yaw: float = 0.0


@dataclass(frozen=True)
class GridSnapshot:
    data: np.ndarray
    info: GridInfo
    frame_id: str = 'map'
    stamp_sec: float | None = None

    def __post_init__(self) -> None:
        if self.data.shape != (self.info.height, self.info.width):
            raise ValueError(
                f'grid shape {self.data.shape} does not match '
                f'{self.info.height}x{self.info.width}'
            )


@dataclass(frozen=True)
class HoughLine:
    theta: float
    rho: float
    votes: int


@dataclass(frozen=True)
class CornerCandidate:
    id: str
    x: float
    y: float
    yaw: float
    wall_intersection_x: float
    wall_intersection_y: float
    clearance_m: float
    distance_from_robot_m: float | None
    score: float
    reachable: bool = True

    def as_dict(self) -> dict[str, Any]:
        result = {
            'id': self.id,
            'x': round(float(self.x), 3),
            'y': round(float(self.y), 3),
            'yaw': round(float(self.yaw), 3),
            'wall_intersection': {
                'x': round(float(self.wall_intersection_x), 3),
                'y': round(float(self.wall_intersection_y), 3),
            },
            'clearance_m': round(float(self.clearance_m), 3),
            'score': round(float(self.score), 3),
            'reachable': self.reachable,
        }
        if self.distance_from_robot_m is not None:
            result['distance_from_robot_m'] = round(float(self.distance_from_robot_m), 3)
        return result


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_to_quaternion(yaw: float) -> tuple[float, float, float, float]:
    half = yaw * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


def yaw_from_quaternion_values(x: float, y: float, z: float, w: float) -> float:
    siny = 2.0 * (w * z + x * y)
    cosy = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny, cosy)


def grid_to_world(info: GridInfo, cell_x: int | float, cell_y: int | float) -> tuple[float, float]:
    local_x = (float(cell_x) + 0.5) * info.resolution
    local_y = (float(cell_y) + 0.5) * info.resolution
    if abs(info.origin_yaw) < 1e-9:
        return info.origin_x + local_x, info.origin_y + local_y
    cos_yaw = math.cos(info.origin_yaw)
    sin_yaw = math.sin(info.origin_yaw)
    return (
        info.origin_x + local_x * cos_yaw - local_y * sin_yaw,
        info.origin_y + local_x * sin_yaw + local_y * cos_yaw,
    )


def world_to_grid(info: GridInfo, x: float, y: float) -> tuple[int, int]:
    dx = float(x) - info.origin_x
    dy = float(y) - info.origin_y
    if abs(info.origin_yaw) >= 1e-9:
        cos_yaw = math.cos(-info.origin_yaw)
        sin_yaw = math.sin(-info.origin_yaw)
        dx, dy = dx * cos_yaw - dy * sin_yaw, dx * sin_yaw + dy * cos_yaw
    return int(math.floor(dx / info.resolution)), int(math.floor(dy / info.resolution))


def in_bounds(info: GridInfo, cell_x: int, cell_y: int) -> bool:
    return 0 <= cell_x < info.width and 0 <= cell_y < info.height


def map_summary(snapshot: GridSnapshot) -> dict[str, Any]:
    data = snapshot.data
    total = max(1, int(data.size))
    unknown = int(np.count_nonzero(data == UNKNOWN))
    free = int(np.count_nonzero((data >= 0) & (data <= DEFAULT_FREE_THRESHOLD)))
    occupied = int(np.count_nonzero(data >= DEFAULT_OCCUPIED_THRESHOLD))
    return {
        'frame_id': snapshot.frame_id,
        'resolution': float(snapshot.info.resolution),
        'width': int(snapshot.info.width),
        'height': int(snapshot.info.height),
        'origin': {
            'x': float(snapshot.info.origin_x),
            'y': float(snapshot.info.origin_y),
            'yaw': float(snapshot.info.origin_yaw),
        },
        'free_percent': round(100.0 * free / total, 2),
        'occupied_percent': round(100.0 * occupied / total, 2),
        'unknown_percent': round(100.0 * unknown / total, 2),
        'stamp_sec': snapshot.stamp_sec,
    }


def classify_occupancy(
    data: np.ndarray,
    free_threshold: int = DEFAULT_FREE_THRESHOLD,
    occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD,
    use_unknown_as_wall: bool = False,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    unknown = data == UNKNOWN
    free = (data >= 0) & (data <= free_threshold)
    occupied = data >= occupied_threshold
    if use_unknown_as_wall:
        occupied = occupied | unknown
    return free, occupied, unknown


def dilate_mask(mask: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return mask.copy()

    result = mask.astype(bool, copy=True)
    for _ in range(radius_cells):
        padded = np.pad(result, 1, mode='constant', constant_values=False)
        result = (
            padded[1:-1, 1:-1]
            | padded[0:-2, 1:-1]
            | padded[2:, 1:-1]
            | padded[1:-1, 0:-2]
            | padded[1:-1, 2:]
            | padded[0:-2, 0:-2]
            | padded[0:-2, 2:]
            | padded[2:, 0:-2]
            | padded[2:, 2:]
        )
    return result


def adjacent_to_mask(mask: np.ndarray, neighbor_mask: np.ndarray) -> np.ndarray:
    padded = np.pad(neighbor_mask.astype(bool), 1, mode='constant', constant_values=False)
    adjacent = (
        padded[0:-2, 1:-1]
        | padded[2:, 1:-1]
        | padded[1:-1, 0:-2]
        | padded[1:-1, 2:]
        | padded[0:-2, 0:-2]
        | padded[0:-2, 2:]
        | padded[2:, 0:-2]
        | padded[2:, 2:]
    )
    return mask.astype(bool) & adjacent


def nearest_cell(mask: np.ndarray, start_x: int, start_y: int, max_radius: int = 50) -> tuple[int, int] | None:
    height, width = mask.shape
    if 0 <= start_x < width and 0 <= start_y < height and mask[start_y, start_x]:
        return start_x, start_y

    for radius in range(1, max_radius + 1):
        x0 = max(0, start_x - radius)
        x1 = min(width - 1, start_x + radius)
        y0 = max(0, start_y - radius)
        y1 = min(height - 1, start_y + radius)
        candidates: list[tuple[float, int, int]] = []
        for x in range(x0, x1 + 1):
            for y in (y0, y1):
                if mask[y, x]:
                    candidates.append((math.hypot(x - start_x, y - start_y), x, y))
        for y in range(y0 + 1, y1):
            for x in (x0, x1):
                if mask[y, x]:
                    candidates.append((math.hypot(x - start_x, y - start_y), x, y))
        if candidates:
            _, cell_x, cell_y = min(candidates)
            return cell_x, cell_y
    return None


def reachable_mask(free_mask: np.ndarray, start_cell: tuple[int, int] | None) -> np.ndarray:
    if start_cell is None:
        return free_mask.astype(bool, copy=True)

    height, width = free_mask.shape
    start_x, start_y = start_cell
    if not (0 <= start_x < width and 0 <= start_y < height):
        return np.zeros_like(free_mask, dtype=bool)
    if not free_mask[start_y, start_x]:
        replacement = nearest_cell(free_mask, start_x, start_y, max_radius=max(width, height))
        if replacement is None:
            return np.zeros_like(free_mask, dtype=bool)
        start_x, start_y = replacement

    reached = np.zeros_like(free_mask, dtype=bool)
    queue: deque[tuple[int, int]] = deque([(start_x, start_y)])
    reached[start_y, start_x] = True
    while queue:
        cell_x, cell_y = queue.popleft()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = cell_x + dx
                ny = cell_y + dy
                if (
                    0 <= nx < width
                    and 0 <= ny < height
                    and not reached[ny, nx]
                    and free_mask[ny, nx]
                ):
                    reached[ny, nx] = True
                    queue.append((nx, ny))
    return reached


def distance_to_nearest_mask(
    mask: np.ndarray,
    cell_x: int,
    cell_y: int,
    resolution: float,
    max_distance_m: float = 3.0,
) -> float | None:
    max_cells = max(1, int(math.ceil(max_distance_m / resolution)))
    height, width = mask.shape
    x0 = max(0, cell_x - max_cells)
    x1 = min(width, cell_x + max_cells + 1)
    y0 = max(0, cell_y - max_cells)
    y1 = min(height, cell_y + max_cells + 1)
    ys, xs = np.where(mask[y0:y1, x0:x1])
    if len(xs) == 0:
        return None
    dx = xs + x0 - cell_x
    dy = ys + y0 - cell_y
    return float(np.min(np.hypot(dx, dy)) * resolution)


def _line_angle_diff(theta_a: float, theta_b: float) -> float:
    diff = abs((theta_a - theta_b) % math.pi)
    return min(diff, math.pi - diff)


def _hough_lines(
    boundary_mask: np.ndarray,
    info: GridInfo,
    theta_step_deg: float = 5.0,
    rho_resolution: float | None = None,
    min_votes: int | None = None,
    max_lines: int = 24,
) -> list[HoughLine]:
    ys, xs = np.where(boundary_mask)
    if len(xs) < 6:
        return []

    rho_resolution = rho_resolution or max(info.resolution * 2.0, 0.05)
    min_votes = min_votes or max(6, int(round(min(info.width, info.height) * 0.08)))

    wx = np.empty(len(xs), dtype=float)
    wy = np.empty(len(xs), dtype=float)
    for i, (cell_x, cell_y) in enumerate(zip(xs, ys)):
        wx[i], wy[i] = grid_to_world(info, int(cell_x), int(cell_y))

    candidates: list[HoughLine] = []
    theta_count = max(1, int(round(180.0 / theta_step_deg)))
    for theta in np.linspace(0.0, math.pi, theta_count, endpoint=False):
        rhos = wx * math.cos(theta) + wy * math.sin(theta)
        bins = np.rint(rhos / rho_resolution).astype(np.int64)
        unique_bins, counts = np.unique(bins, return_counts=True)
        for bin_value, count in zip(unique_bins, counts):
            if int(count) >= min_votes:
                candidates.append(HoughLine(theta=theta, rho=float(bin_value) * rho_resolution, votes=int(count)))

    candidates.sort(key=lambda line: line.votes, reverse=True)
    merged: list[HoughLine] = []
    for line in candidates:
        duplicate_index = None
        for idx, existing in enumerate(merged):
            if _line_angle_diff(line.theta, existing.theta) <= math.radians(theta_step_deg * 1.5):
                if abs(line.rho - existing.rho) <= rho_resolution * 2.0:
                    duplicate_index = idx
                    break
        if duplicate_index is None:
            merged.append(line)
        elif line.votes > merged[duplicate_index].votes:
            merged[duplicate_index] = line
        if len(merged) >= max_lines:
            break
    return merged


def _line_intersection(line_a: HoughLine, line_b: HoughLine) -> tuple[float, float] | None:
    cos_a = math.cos(line_a.theta)
    sin_a = math.sin(line_a.theta)
    cos_b = math.cos(line_b.theta)
    sin_b = math.sin(line_b.theta)
    det = cos_a * sin_b - sin_a * cos_b
    if abs(det) < 1e-6:
        return None
    x = (line_a.rho * sin_b - line_b.rho * sin_a) / det
    y = (cos_a * line_b.rho - cos_b * line_a.rho) / det
    return x, y


def _line_distance(line: HoughLine, x: float, y: float) -> float:
    return abs(x * math.cos(line.theta) + y * math.sin(line.theta) - line.rho)


def _candidate_sort_key(policy: str, candidate: CornerCandidate) -> tuple[float, float]:
    distance = candidate.distance_from_robot_m
    if policy == 'nearest' and distance is not None:
        return distance, -candidate.score
    if policy == 'farthest' and distance is not None:
        return -distance, -candidate.score
    return -candidate.score, distance if distance is not None else 0.0


def _dedupe_candidates(candidates: Iterable[CornerCandidate], min_separation_m: float) -> list[CornerCandidate]:
    result: list[CornerCandidate] = []
    for candidate in sorted(candidates, key=lambda item: item.score, reverse=True):
        too_close = False
        for existing in result:
            if math.hypot(candidate.x - existing.x, candidate.y - existing.y) < min_separation_m:
                too_close = True
                break
        if not too_close:
            result.append(candidate)
    return result


def _sample_corner_goal(
    line_a: HoughLine,
    line_b: HoughLine,
    intersection: tuple[float, float],
    safe_mask: np.ndarray,
    wall_mask: np.ndarray,
    info: GridInfo,
    robot_xy: tuple[float, float] | None,
    min_clearance_m: float,
    max_corner_offset_m: float,
    angle_score: float,
    support_score: float,
) -> CornerCandidate | None:
    ix, iy = intersection
    preferred_offset = min(max_corner_offset_m, max(min_clearance_m * 1.35, min_clearance_m + 0.15))
    best: CornerCandidate | None = None

    radii = np.linspace(min_clearance_m, max_corner_offset_m, num=7)
    for radius in radii:
        for theta in np.linspace(0.0, 2.0 * math.pi, num=72, endpoint=False):
            x = ix + float(radius) * math.cos(theta)
            y = iy + float(radius) * math.sin(theta)
            cell_x, cell_y = world_to_grid(info, x, y)
            if not in_bounds(info, cell_x, cell_y):
                continue
            if not safe_mask[cell_y, cell_x]:
                continue

            line_dist_a = _line_distance(line_a, x, y)
            line_dist_b = _line_distance(line_b, x, y)
            if line_dist_a < min_clearance_m * 0.75 or line_dist_b < min_clearance_m * 0.75:
                continue
            if line_dist_a > max_corner_offset_m * 1.25 or line_dist_b > max_corner_offset_m * 1.25:
                continue

            clearance = distance_to_nearest_mask(
                wall_mask,
                cell_x,
                cell_y,
                info.resolution,
                max_distance_m=max_corner_offset_m * 2.0,
            )
            if clearance is None or clearance < min_clearance_m:
                continue

            offset_error = abs((line_dist_a + line_dist_b) * 0.5 - preferred_offset)
            distance_score = max(0.0, 1.0 - offset_error / max(preferred_offset, 1e-6))
            clearance_score = min(clearance / max(min_clearance_m, 1e-6), 2.0) / 2.0
            robot_distance = None
            robot_score = 0.0
            if robot_xy is not None:
                robot_distance = math.hypot(x - robot_xy[0], y - robot_xy[1])
                robot_score = 1.0 / (1.0 + robot_distance)

            score = (
                0.45 * angle_score
                + 0.25 * support_score
                + 0.15 * distance_score
                + 0.10 * clearance_score
                + 0.05 * robot_score
            )
            yaw = math.atan2(iy - y, ix - x)
            candidate = CornerCandidate(
                id='corner',
                x=x,
                y=y,
                yaw=yaw,
                wall_intersection_x=ix,
                wall_intersection_y=iy,
                clearance_m=clearance,
                distance_from_robot_m=robot_distance,
                score=score,
            )
            if best is None or candidate.score > best.score:
                best = candidate
    return best


def detect_corners(
    snapshot: GridSnapshot,
    number_of_corners_required: int = 4,
    robot_xy: tuple[float, float] | None = None,
    policy: str = 'nearest',
    min_clearance_m: float = 0.45,
    max_corner_offset_m: float = 1.2,
    use_unknown_as_wall: bool = False,
    free_threshold: int = DEFAULT_FREE_THRESHOLD,
    occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD,
) -> dict[str, Any]:
    policy = (policy or 'nearest').lower()
    if policy not in {'nearest', 'farthest', 'best', 'all'}:
        policy = 'nearest'

    info = snapshot.info
    data = snapshot.data
    free_mask, wall_mask, _unknown = classify_occupancy(
        data,
        free_threshold=free_threshold,
        occupied_threshold=occupied_threshold,
        use_unknown_as_wall=use_unknown_as_wall,
    )

    start_cell = None
    if robot_xy is not None:
        start_cell = world_to_grid(info, robot_xy[0], robot_xy[1])

    reachable = reachable_mask(free_mask, start_cell)
    clearance_cells = max(1, int(math.ceil(min_clearance_m / info.resolution)))
    inflated_walls = dilate_mask(wall_mask, clearance_cells)
    safe_mask = reachable & free_mask & ~inflated_walls
    boundary = adjacent_to_mask(reachable & free_mask, wall_mask)
    lines = _hough_lines(boundary, info)

    raw_candidates: list[CornerCandidate] = []
    angle_tolerance = math.radians(25.0)
    for idx, line_a in enumerate(lines):
        for line_b in lines[idx + 1:]:
            angle_diff = _line_angle_diff(line_a.theta, line_b.theta)
            right_angle_error = abs(angle_diff - math.pi / 2.0)
            if right_angle_error > angle_tolerance:
                continue
            intersection = _line_intersection(line_a, line_b)
            if intersection is None:
                continue
            inter_cell = world_to_grid(info, intersection[0], intersection[1])
            margin = int(math.ceil(max_corner_offset_m / info.resolution)) + 2
            if not (
                -margin <= inter_cell[0] < info.width + margin
                and -margin <= inter_cell[1] < info.height + margin
            ):
                continue

            angle_score = max(0.0, 1.0 - right_angle_error / angle_tolerance)
            support_score = min(math.sqrt(line_a.votes * line_b.votes) / max(info.width, info.height), 1.0)
            candidate = _sample_corner_goal(
                line_a=line_a,
                line_b=line_b,
                intersection=intersection,
                safe_mask=safe_mask,
                wall_mask=wall_mask,
                info=info,
                robot_xy=robot_xy,
                min_clearance_m=min_clearance_m,
                max_corner_offset_m=max_corner_offset_m,
                angle_score=angle_score,
                support_score=support_score,
            )
            if candidate is not None:
                raw_candidates.append(candidate)

    min_sep = max(min_clearance_m, max_corner_offset_m * 0.5)
    candidates = _dedupe_candidates(raw_candidates, min_sep)
    candidates.sort(key=lambda item: _candidate_sort_key(policy, item))
    if policy != 'all':
        candidates = candidates[:max(0, int(number_of_corners_required))]

    numbered = [
        CornerCandidate(
            id=f'corner_{index + 1}',
            x=candidate.x,
            y=candidate.y,
            yaw=candidate.yaw,
            wall_intersection_x=candidate.wall_intersection_x,
            wall_intersection_y=candidate.wall_intersection_y,
            clearance_m=candidate.clearance_m,
            distance_from_robot_m=candidate.distance_from_robot_m,
            score=candidate.score,
            reachable=candidate.reachable,
        )
        for index, candidate in enumerate(candidates)
    ]

    return {
        'frame_id': snapshot.frame_id,
        'count': len(numbered),
        'corners': [candidate.as_dict() for candidate in numbered],
        'debug': {
            'line_count': len(lines),
            'raw_candidate_count': len(raw_candidates),
            'safe_cell_count': int(np.count_nonzero(safe_mask)),
            'boundary_cell_count': int(np.count_nonzero(boundary)),
            'policy': policy,
            'min_clearance_m': float(min_clearance_m),
            'use_unknown_as_wall': bool(use_unknown_as_wall),
        },
    }


def validate_goal(
    snapshot: GridSnapshot,
    x: float,
    y: float,
    min_clearance_m: float = 0.45,
    robot_xy: tuple[float, float] | None = None,
    costmap_snapshot: GridSnapshot | None = None,
    allow_unknown: bool = False,
    free_threshold: int = DEFAULT_FREE_THRESHOLD,
    occupied_threshold: int = DEFAULT_OCCUPIED_THRESHOLD,
) -> dict[str, Any]:
    check_snapshot = costmap_snapshot or snapshot
    info = check_snapshot.info
    data = check_snapshot.data
    cell_x, cell_y = world_to_grid(info, x, y)
    if not in_bounds(info, cell_x, cell_y):
        return {
            'valid': False,
            'reason': 'outside_grid',
            'frame_id': check_snapshot.frame_id,
            'cell': {'x': cell_x, 'y': cell_y},
        }

    value = int(data[cell_y, cell_x])
    if value == UNKNOWN and not allow_unknown:
        return {
            'valid': False,
            'reason': 'unknown_cell',
            'frame_id': check_snapshot.frame_id,
            'cell': {'x': cell_x, 'y': cell_y},
            'value': value,
        }
    if value >= occupied_threshold:
        return {
            'valid': False,
            'reason': 'occupied_cell',
            'frame_id': check_snapshot.frame_id,
            'cell': {'x': cell_x, 'y': cell_y},
            'value': value,
        }
    if value > free_threshold and value != UNKNOWN:
        return {
            'valid': False,
            'reason': 'inflated_or_high_cost_cell',
            'frame_id': check_snapshot.frame_id,
            'cell': {'x': cell_x, 'y': cell_y},
            'value': value,
        }

    source_free, source_walls, _source_unknown = classify_occupancy(
        snapshot.data,
        free_threshold=free_threshold,
        occupied_threshold=occupied_threshold,
    )
    source_cell_x, source_cell_y = world_to_grid(snapshot.info, x, y)
    if not in_bounds(snapshot.info, source_cell_x, source_cell_y):
        return {
            'valid': False,
            'reason': 'outside_map',
            'frame_id': snapshot.frame_id,
        }

    clearance = distance_to_nearest_mask(
        source_walls,
        source_cell_x,
        source_cell_y,
        snapshot.info.resolution,
        max_distance_m=max(2.0, min_clearance_m * 3.0),
    )
    if clearance is not None and clearance < min_clearance_m:
        return {
            'valid': False,
            'reason': 'insufficient_clearance',
            'frame_id': snapshot.frame_id,
            'cell': {'x': source_cell_x, 'y': source_cell_y},
            'value': int(snapshot.data[source_cell_y, source_cell_x]),
            'clearance_m': round(float(clearance), 3),
            'required_clearance_m': float(min_clearance_m),
        }

    reachable = None
    if robot_xy is not None:
        start_cell = world_to_grid(snapshot.info, robot_xy[0], robot_xy[1])
        reachable_area = reachable_mask(source_free, start_cell)
        reachable = bool(reachable_area[source_cell_y, source_cell_x])
        if not reachable:
            return {
                'valid': False,
                'reason': 'not_reachable_from_robot_region',
                'frame_id': snapshot.frame_id,
                'cell': {'x': source_cell_x, 'y': source_cell_y},
            }

    return {
        'valid': True,
        'reason': 'reachable_free_space' if reachable else 'free_space',
        'frame_id': snapshot.frame_id,
        'cell': {'x': source_cell_x, 'y': source_cell_y},
        'value': int(snapshot.data[source_cell_y, source_cell_x]),
        'costmap_value': value if costmap_snapshot is not None else None,
        'clearance_m': None if clearance is None else round(float(clearance), 3),
        'required_clearance_m': float(min_clearance_m),
    }
