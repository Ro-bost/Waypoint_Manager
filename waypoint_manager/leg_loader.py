"""Load per-leg waypoint YAML files and build routes from device diagnostics."""

from __future__ import annotations

import math
import numbers
from pathlib import Path
from typing import Any, List, Mapping, Optional, Sequence, Tuple

from geometry_msgs.msg import PoseStamped, Quaternion

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

DEVICE_NAMES = ("device1", "device2", "device3", "device4")


def diagnostic_level_to_int(level: Any) -> int:
    """Convert DiagnosticStatus.level to int (Humble bytes, Foxy int/uint8)."""
    if isinstance(level, numbers.Integral) and not isinstance(level, bool):
        return int(level)
    if isinstance(level, (bytes, bytearray)):
        return int(level[0]) if level else 0
    return int(level)


def parse_vertex_from_name(name: str) -> Optional[int]:
    """Map status[].name to map vertex 1-4 (ICROS: device1 .. device4)."""
    cleaned = name.strip()
    if not cleaned.startswith("device") or len(cleaned) <= 6:
        return None
    suffix = cleaned[6:]
    if not suffix.isdigit():
        return None
    vertex = int(suffix)
    if 1 <= vertex <= 4:
        return vertex
    return None


def get_target_vertex_from_statuses(statuses: Sequence[Any]) -> Optional[int]:
    """Return the next visit vertex from a live diagnostics message.

    Operations send targets one at a time over 1Hz updates (e.g. only device3
    non-zero, later only device2, finally device1 for return). The first
    status[] entry with level != 0 wins.
    """
    for status in statuses:
        level = diagnostic_level_to_int(getattr(status, "level", 0))
        if level == 0:
            continue

        vertex = parse_vertex_from_name(str(getattr(status, "name", "")))
        if vertex is not None:
            return vertex

    return None


def build_hop_route(current_vertex: int, target_vertex: int) -> List[int]:
    """Single leg: current position vertex -> next target vertex."""
    if current_vertex == target_vertex:
        return [current_vertex]
    return [current_vertex, target_vertex]


def build_visit_route_from_statuses(
    statuses: Sequence[Any],
    *,
    home_vertex: int = 1,
) -> List[int]:
    """Legacy helper: expand one live message into a multi-stop route (tests only)."""
    target = get_target_vertex_from_statuses(statuses)
    if target is None:
        return [home_vertex]
    return build_hop_route(home_vertex, target)


def build_visit_route(
    device_levels: Mapping[str, int],
    *,
    home_vertex: int = 1,
) -> List[int]:
    """Build a route from a name->level map (used in tests and logging)."""

    class _Status:
        def __init__(self, name: str, level: int) -> None:
            self.name = name
            self.level = level

    statuses = [
        _Status(name, int(device_levels.get(name, 0))) for name in DEVICE_NAMES
    ]
    return build_visit_route_from_statuses(statuses, home_vertex=home_vertex)


def route_to_leg_pairs(route: Sequence[int]) -> List[Tuple[int, int]]:
    if len(route) < 2:
        return []
    return [(route[i], route[i + 1]) for i in range(len(route) - 1)]


def leg_filename(from_vertex: int, to_vertex: int) -> str:
    return f"{from_vertex}_to_{to_vertex}.yaml"


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def pose_from_xy_yaw(frame_id: str, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation = yaw_to_quat(yaw)
    return pose


def load_leg_file(path: Path, default_frame_id: str) -> Tuple[str, List[PoseStamped]]:
    if yaml is None:
        raise RuntimeError("PyYAML is not installed")

    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)

    if not isinstance(data, dict):
        raise ValueError(f"Leg file must be a YAML mapping: {path}")

    frame_id = str(data.get("frame_id", default_frame_id))
    entries = data.get("waypoints", [])
    if not entries:
        raise ValueError(f"No waypoints in leg file: {path}")

    poses: List[PoseStamped] = []
    for idx, entry in enumerate(entries):
        try:
            x = float(entry["x"])
            y = float(entry["y"])
            yaw = float(entry.get("yaw", 0.0))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"Invalid waypoint #{idx + 1} in {path}: {exc}") from exc
        poses.append(pose_from_xy_yaw(frame_id, x, y, yaw))

    return frame_id, poses


def load_route_waypoints(
    route: Sequence[int],
    legs_dir: Path,
    default_frame_id: str,
) -> Tuple[str, List[PoseStamped]]:
    """Concatenate leg YAMLs for each consecutive pair in route."""
    pairs = route_to_leg_pairs(route)
    if not pairs:
        raise ValueError("Route has no legs to load")

    merged: List[PoseStamped] = []
    frame_id = default_frame_id

    for from_vertex, to_vertex in pairs:
        path = legs_dir / leg_filename(from_vertex, to_vertex)
        if not path.is_file():
            raise FileNotFoundError(f"Missing leg YAML: {path}")

        leg_frame_id, leg_poses = load_leg_file(path, default_frame_id)
        frame_id = leg_frame_id
        merged.extend(leg_poses)

    return frame_id, merged
