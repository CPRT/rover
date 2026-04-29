import math
from functools import lru_cache
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import yaml
from geometry_msgs.msg import PoseStamped

try:
    # Package import path (normal ROS node runtime)
    from .gps_utils import quaternion_from_euler
except ImportError:
    try:
        # Absolute package path (when run as module)
        from nav_commanders.gps_utils import quaternion_from_euler
    except ImportError:
        # Local file import (when run directly as a script)
        from gps_utils import quaternion_from_euler


CONFIG_FILENAME = "search_patterns.yaml"
PACKAGE_NAME = "nav_commanders"


def _normalize_pattern_name(pattern: str) -> str:
    return pattern.strip().lower().replace("-", "_")


def _candidate_config_paths(filename: str = CONFIG_FILENAME) -> List[Path]:
    module_path = Path(__file__).resolve()
    candidates: List[Path] = []
    seen = set()

    for ancestor in [module_path.parent, *module_path.parents]:
        for candidate in (
            ancestor / "config" / filename,
            ancestor / "share" / PACKAGE_NAME / "config" / filename,
        ):
            if candidate in seen:
                continue
            seen.add(candidate)
            candidates.append(candidate)

    return candidates


@lru_cache(maxsize=1)
def load_search_pattern_config() -> dict:
    for candidate in _candidate_config_paths():
        if not candidate.is_file():
            continue

        with candidate.open("r", encoding="utf-8") as stream:
            data = yaml.safe_load(stream) or {}

        if not isinstance(data, dict):
            raise ValueError(f"{candidate} must contain a mapping of patterns")

        normalized = {}
        for key, value in data.items():
            if not isinstance(key, str):
                raise ValueError(f"{candidate} has a non-string pattern key: {key!r}")
            if not isinstance(value, dict):
                raise ValueError(
                    f"{candidate} entry for '{key}' must be a mapping of options"
                )
            normalized[_normalize_pattern_name(key)] = value

        return normalized

    searched = ", ".join(str(path) for path in _candidate_config_paths())
    raise FileNotFoundError(f"Could not find {CONFIG_FILENAME}. Searched: {searched}")


def _resolve_pattern_config(pattern: str) -> dict:
    config = load_search_pattern_config()
    key = _normalize_pattern_name(pattern)

    if key not in config:
        available = ", ".join(sorted(config.keys()))
        raise KeyError(f"Unknown search pattern '{pattern}'. Available: {available}")

    return dict(config[key])


def _resolve_pattern_config_by_radius(pattern: str, search_radius: float) -> dict:
    config = load_search_pattern_config()
    normalized_pattern = _normalize_pattern_name(pattern)

    for value in config.values():
        if _normalize_pattern_name(str(value.get("pattern", ""))) != normalized_pattern:
            continue

        config_search_radius = value.get("search_radius")
        if config_search_radius is None:
            continue

        if math.isclose(
            float(config_search_radius), float(search_radius), rel_tol=0.0, abs_tol=1e-6
        ):
            return dict(value)

    available = ", ".join(sorted(config.keys()))
    raise KeyError(
        f"Unknown search pattern '{pattern}' with search_radius={search_radius}. Available: {available}"
    )


def archimedean_spiral_xy(
    center_x: float,
    center_y: float,
    distance_between_loops: float = 2.0,
    num_points: int = 50,
    arc_length_step: float = 0.8,
    start_theta: float = 0.0,
) -> List[Tuple[float, float, float, float]]:
    """
    Generate Archimedean spiral samples in 2D.

    Returns tuples of (x, y, theta, radius).
    The radius model is r = b * theta where b = distance_between_loops / (2*pi).
    """
    if num_points <= 0:
        return []
    if distance_between_loops <= 0.0:
        raise ValueError("distance_between_loops must be > 0")
    if arc_length_step <= 0.0:
        raise ValueError("arc_length_step must be > 0")

    b = distance_between_loops / (2.0 * math.pi)
    samples: List[Tuple[float, float, float, float]] = []

    theta = start_theta
    for _ in range(num_points):
        ds_dtheta = b * math.sqrt((theta * theta) + 1.0)
        if ds_dtheta <= 0.0:
            break
        delta_theta = arc_length_step / ds_dtheta
        theta += delta_theta
        radius = b * theta
        x = center_x + radius * math.cos(theta)
        y = center_y + radius * math.sin(theta)
        samples.append((x, y, theta, radius))

    return samples


def generate_search_spiral(
    center_x: float,
    center_y: float,
    distance_between_loops: float = 2.0,
    num_points: int = 50,
    frame_id: str = "map",
    arc_length_step: float = 0.8,
    start_theta: float = 0.0,
    face_outward: bool = False,
    min_radius: float = 0.0,
) -> List[PoseStamped]:
    """
    Generate PoseStamped waypoints on an Archimedean spiral.

    Orientation is aligned with motion tangent by default. If face_outward is True,
    orientation instead points radially away from the spiral center.
    Points inside min_radius are skipped to avoid tight inner loops.
    Points are spaced by approximate arc length using arc_length_step.
    """
    if min_radius < 0.0:
        raise ValueError("min_radius must be >= 0")
    if arc_length_step <= 0.0:
        raise ValueError("arc_length_step must be > 0")

    samples = archimedean_spiral_xy(
        center_x=center_x,
        center_y=center_y,
        distance_between_loops=distance_between_loops,
        num_points=num_points,
        arc_length_step=arc_length_step,
        start_theta=start_theta,
    )

    poses: List[PoseStamped] = []
    for x, y, theta, radius in samples:
        if radius < min_radius:
            continue

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0

        # Tangent heading for r=b*theta is atan2(r' sin(theta)+r cos(theta),
        # r' cos(theta)-r sin(theta)). The simple theta+pi/2 approximation works,
        # but this exact expression is slightly better on inner loops.
        if face_outward:
            yaw = theta
        else:
            dr_dtheta = distance_between_loops / (2.0 * math.pi)
            dx_dtheta = dr_dtheta * math.cos(theta) - (dr_dtheta * theta) * math.sin(
                theta
            )
            dy_dtheta = dr_dtheta * math.sin(theta) + (dr_dtheta * theta) * math.cos(
                theta
            )
            yaw = math.atan2(dy_dtheta, dx_dtheta)

        q = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation = q
        poses.append(pose)

    return poses


def generate_search_lawnmower(
    center_x: float,
    center_y: float,
    search_size: float = 20.0,
    lane_spacing: float = 4.0,
    frame_id: str = "map",
) -> List[PoseStamped]:
    """
    Generate PoseStamped waypoints for a rectilinear (lawnmower/boustrophedon) pattern.

    The pattern covers a square of side `search_size` centered at (center_x, center_y),
    with parallel lanes separated by `lane_spacing`. Orientations alternate between 0 and pi
    so heading points along the lane direction.
    """
    if lane_spacing <= 0.0:
        raise ValueError("lane_spacing must be > 0")

    half_size = search_size / 2.0
    start_x = center_x - half_size
    start_y = center_y - half_size
    num_lanes = int(search_size / lane_spacing) + 1

    poses: List[PoseStamped] = []
    for i in range(num_lanes):
        current_y = start_y + (i * lane_spacing)

        if i % 2 == 0:
            x_start, x_end, yaw = start_x, start_x + search_size, 0.0
        else:
            x_start, x_end, yaw = start_x + search_size, start_x, math.pi

        pose_start = PoseStamped()
        pose_start.header.frame_id = frame_id
        pose_start.pose.position.x = float(x_start)
        pose_start.pose.position.y = float(current_y)
        pose_start.pose.position.z = 0.0
        pose_start.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
        poses.append(pose_start)

        pose_end = PoseStamped()
        pose_end.header.frame_id = frame_id
        pose_end.pose.position.x = float(x_end)
        pose_end.pose.position.y = float(current_y)
        pose_end.pose.position.z = 0.0
        pose_end.pose.orientation = quaternion_from_euler(0.0, 0.0, yaw)
        poses.append(pose_end)

    return poses


def generate_search_pattern(
    pattern: str,
    center_x: float,
    center_y: float,
    frame_id: str = "map",
    **kwargs,
) -> List[PoseStamped]:
    """
    Helper to select and generate a search pattern.

    Pattern specs are loaded from config/search_patterns.yaml.
    Additional keyword arguments override the config entry.
    """
    try:
        pattern_config = _resolve_pattern_config(pattern)
    except KeyError:
        search_radius = kwargs.get("search_radius")
        if _normalize_pattern_name(pattern) == "spiral" and search_radius is not None:
            pattern_config = _resolve_pattern_config_by_radius(pattern, search_radius)
        else:
            raise

    pattern_type = _normalize_pattern_name(pattern_config.pop("pattern", pattern))
    pattern_config.update(kwargs)

    if pattern_type == "spiral":
        search_radius = pattern_config.pop("search_radius", None)
        distance_between_loops = float(
            pattern_config.get("distance_between_loops", 2.0)
        )
        arc_length_step = float(pattern_config.get("arc_length_step", 0.8))
        start_theta = float(pattern_config.get("start_theta", 0.0))

        if search_radius is not None:
            if distance_between_loops <= 0.0:
                raise ValueError("distance_between_loops must be > 0")
            if arc_length_step <= 0.0:
                raise ValueError("arc_length_step must be > 0")

            b = distance_between_loops / (2.0 * math.pi)
            theta_final = float(search_radius) / b
            def spiral_arc_length(theta: float) -> float:
                return 0.5 * b * (
                    theta * math.sqrt((theta * theta) + 1.0) + math.asinh(theta)
                )

            delta_s = spiral_arc_length(theta_final) - spiral_arc_length(start_theta)
            num_points = int(math.ceil(max(0.0, delta_s) / arc_length_step))
            pattern_config["num_points"] = max(1, num_points)

        pattern_config.pop("search_radius", None)
        pattern_config["arc_length_step"] = arc_length_step
        return generate_search_spiral(
            center_x=center_x,
            center_y=center_y,
            frame_id=frame_id,
            **pattern_config,
        )

    if pattern_type in ("lawnmower", "rectilinear", "boustrophedon"):
        return generate_search_lawnmower(
            center_x=center_x,
            center_y=center_y,
            frame_id=frame_id,
            **pattern_config,
        )

    raise ValueError(f"Unknown pattern type '{pattern_type}' in config for '{pattern}'")


def extract_xy(poses: Sequence[PoseStamped]) -> Tuple[List[float], List[float]]:
    """Return x and y arrays from PoseStamped sequence."""
    xs = [pose.pose.position.x for pose in poses]
    ys = [pose.pose.position.y for pose in poses]
    return xs, ys


def plot_spirals(
    spirals: Sequence[Tuple[str, Sequence[PoseStamped]]],
    center: Optional[Tuple[float, float]] = None,
    show_arrows_every: int = 5,
) -> None:
    """
    Plot one or more spiral waypoint sets using matplotlib.

    Each element in spirals is (label, poses).
    """
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        print("matplotlib not available, cannot render plot")
        print(f"reason: {exc}")
        return

    fig, ax = plt.subplots(figsize=(8, 8))

    for label, poses in spirals:
        if not poses:
            continue

        xs, ys = extract_xy(poses)
        ax.plot(xs, ys, marker="o", markersize=3, linewidth=1.2, label=label)

        if show_arrows_every > 0:
            for idx in range(0, len(poses), show_arrows_every):
                p = poses[idx]
                q = p.pose.orientation
                # yaw from quaternion (z,w only needed for planar case)
                yaw = math.atan2(2.0 * (q.w * q.z), 1.0 - 2.0 * (q.z * q.z))
                dx = 0.4 * math.cos(yaw)
                dy = 0.4 * math.sin(yaw)
                ax.arrow(
                    p.pose.position.x,
                    p.pose.position.y,
                    dx,
                    dy,
                    head_width=0.12,
                    head_length=0.16,
                    length_includes_head=True,
                    alpha=0.7,
                )

    if center is not None:
        ax.plot(center[0], center[1], "x", markersize=8, label="center")

    ax.set_title("Archimedean Spiral Search Waypoints")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, linestyle="--", linewidth=0.6, alpha=0.6)
    ax.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    cx = 0.0
    cy = 0.0

    config = load_search_pattern_config()
    plotted_patterns = []

    for pattern_name in sorted(config.keys()):
        poses = generate_search_pattern(pattern_name, center_x=cx, center_y=cy)
        print(f"{pattern_name} points: {len(poses)}")
        plotted_patterns.append((pattern_name, poses))

    plot_spirals(
        plotted_patterns,
        center=(cx, cy),
        show_arrows_every=6,
    )
