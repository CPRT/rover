import math
from typing import List, Optional, Sequence, Tuple

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


def archimedean_spiral_xy(
	center_x: float,
	center_y: float,
	distance_between_loops: float = 2.0,
	num_points: int = 50,
	angle_step: float = 0.5,
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
	if angle_step <= 0.0:
		raise ValueError("angle_step must be > 0")

	b = distance_between_loops / (2.0 * math.pi)
	samples: List[Tuple[float, float, float, float]] = []

	for i in range(1, num_points + 1):
		theta = start_theta + i * angle_step
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
	angle_step: float = 0.5,
	start_theta: float = 0.0,
	face_outward: bool = False,
) -> List[PoseStamped]:
	"""
	Generate PoseStamped waypoints on an Archimedean spiral.

	Orientation is aligned with motion tangent by default. If face_outward is True,
	orientation instead points radially away from the spiral center.
	"""
	samples = archimedean_spiral_xy(
		center_x=center_x,
		center_y=center_y,
		distance_between_loops=distance_between_loops,
		num_points=num_points,
		angle_step=angle_step,
		start_theta=start_theta,
	)

	poses: List[PoseStamped] = []
	for x, y, theta, _radius in samples:
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
			dx_dtheta = dr_dtheta * math.cos(theta) - (dr_dtheta * theta) * math.sin(theta)
			dy_dtheta = dr_dtheta * math.sin(theta) + (dr_dtheta * theta) * math.cos(theta)
			yaw = math.atan2(dy_dtheta, dx_dtheta)

		q = quaternion_from_euler(0.0, 0.0, yaw)
		pose.pose.orientation = q
		poses.append(pose)

	return poses


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

	spiral_a = generate_search_spiral(
		center_x=cx,
		center_y=cy,
		distance_between_loops=2.0,
		num_points=40,
		angle_step=0.45,
	)
	spiral_b = generate_search_spiral(
		center_x=cx,
		center_y=cy,
		distance_between_loops=3.0,
		num_points=40,
		angle_step=0.45,
	)
	spiral_c = generate_search_spiral(
		center_x=cx,
		center_y=cy,
		distance_between_loops=2.0,
		num_points=70,
		angle_step=0.30,
	)

	print(f"Spiral A points: {len(spiral_a)}")
	print(f"Spiral B points: {len(spiral_b)}")
	print(f"Spiral C points: {len(spiral_c)}")

	plot_spirals(
		[
			("loops=2.0m, points=40, step=0.45", spiral_a),
			("loops=3.0m, points=40, step=0.45", spiral_b),
			("loops=2.0m, points=70, step=0.30", spiral_c),
		],
		center=(cx, cy),
		show_arrows_every=6,
	)
