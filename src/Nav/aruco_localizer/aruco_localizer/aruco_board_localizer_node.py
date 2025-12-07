#!/usr/bin/env python3
"""
ArUco Board Localizer Node

This node uses detected ArUco boards to provide robot localization estimates.
It maintains a database of known board locations in the map frame and uses
newly detected boards to estimate the robot's pose.

Workflow:
1. Subscribe to /aruco_boards for board detections (in camera frame)
2. Use TF tree to get robot position (map -> odom -> base_link -> camera_link)
3. When a board is first seen, accumulate samples to establish location
4. Once a board location is validated, store it as "known"
5. Use known boards + new detections to publish robot pose estimates

Architecture Note:
- This node uses TF as the source of truth for robot localization
- The TF tree (map -> odom -> base_link -> camera_link) provides all needed transforms
- No direct odometry subscription needed - TF handles frame relationships

Author: GitHub Copilot
Version: 2025-12-06
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import numpy as np
from collections import defaultdict
from dataclasses import dataclass, field
from typing import Dict, List, Optional
import threading
from scipy.spatial.transform import Rotation

# ROS2 message imports
from interfaces.msg import ArucoBoard
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose, TransformStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf2_ros import (
    Buffer,
    TransformListener,
    LookupException,
    ConnectivityException,
    ExtrapolationException,
)
import tf2_geometry_msgs


@dataclass
class BoardSample:
    """Represents a single observation of a board's position in map frame"""

    timestamp: Time
    position: np.ndarray  # [x, y, z]
    orientation: np.ndarray  # quaternion [x, y, z, w]


@dataclass
class BoardCandidate:
    """Tracks samples for a board that's being validated"""

    board_id: str
    samples: List[BoardSample] = field(default_factory=list)

    def add_sample(self, sample: BoardSample):
        self.samples.append(sample)

    def get_recent_samples(
        self, current_time: Time, window_duration: Duration
    ) -> List[BoardSample]:
        """Get samples within the time window"""
        cutoff_time = current_time - window_duration
        return [s for s in self.samples if s.timestamp >= cutoff_time]

    def compute_statistics(self, samples: List[BoardSample]) -> tuple:
        """Compute mean position and standard deviation"""
        if not samples:
            return None, None

        positions = np.array([s.position for s in samples])
        mean_pos = np.mean(positions, axis=0)
        std_pos = np.std(positions, axis=0)
        max_std = np.max(std_pos)

        # For orientation, we'll use the most recent one (proper averaging requires quaternion slerp)
        mean_orientation = samples[-1].orientation

        return mean_pos, mean_orientation, max_std


@dataclass
class KnownBoard:
    """Represents a validated board with known location in map frame"""

    board_id: str
    position: np.ndarray  # [x, y, z]
    orientation: np.ndarray  # quaternion [x, y, z, w]
    num_observations: int = 0
    last_seen: Optional[Time] = None


class ArucoBoardLocalizerNode(Node):
    def __init__(self):
        super().__init__("aruco_board_localizer_node")

        self.initialize_parameters()

        # TF2 Buffer and Listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Board tracking
        self.known_boards: Dict[str, KnownBoard] = {}
        self.candidate_boards: Dict[str, BoardCandidate] = {}
        self.lock = threading.Lock()

        # Bootstrap tracking
        self.last_bootstrap_time: Optional[Time] = None
        self.bootstrap_published_count = 0

        # Subscribers
        self.aruco_sub = self.create_subscription(
            ArucoBoard, self.aruco_topic, self.aruco_callback, 10
        )

        # Publisher
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, self.output_topic, 10
        )

        # Marker publishers
        self.marker_pub = self.create_publisher(MarkerArray, "/aruco_board_markers", 10)

        # Timer for cleanup of old candidates
        self.cleanup_timer = self.create_timer(
            self.candidate_cleanup_interval, self.cleanup_old_candidates
        )

        self.get_logger().info("ArUco Board Localizer Node initialized")
        self.get_logger().info(f"Listening for boards on: {self.aruco_topic}")
        self.get_logger().info(f"Publishing pose estimates to: {self.output_topic}")
        self.get_logger().info(
            f"Using TF for robot localization: {self.map_frame} -> {self.base_link_frame}"
        )

    def initialize_parameters(self):
        """Declare and read ROS2 parameters"""

        # Topics
        self.declare_parameter(
            name="aruco_topic",
            value="/aruco_boards",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to subscribe to for ArUco board detections",
            ),
        )

        self.declare_parameter(
            name="output_topic",
            value="/aruco_pose_estimate",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish robot pose estimates",
            ),
        )

        # Frame IDs
        self.declare_parameter(
            name="map_frame",
            value="map",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING, description="Map frame ID"
            ),
        )

        self.declare_parameter(
            name="base_link_frame",
            value="base_link",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING, description="Robot base frame ID"
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="zed_camera_link",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING, description="Camera frame ID"
            ),
        )

        # Board validation parameters
        self.declare_parameter(
            name="min_samples_for_validation",
            value=10,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Minimum number of samples needed to validate a board",
            ),
        )

        self.declare_parameter(
            name="validation_window_seconds",
            value=2.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Time window in seconds for collecting validation samples",
            ),
        )

        self.declare_parameter(
            name="max_position_std_meters",
            value=0.3,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Maximum position standard deviation (meters) to validate board",
            ),
        )

        self.declare_parameter(
            name="candidate_timeout_seconds",
            value=10.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Time before removing stale candidate boards",
            ),
        )

        self.declare_parameter(
            name="candidate_cleanup_interval",
            value=5.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Interval for cleanup timer in seconds",
            ),
        )

        # Covariance parameters
        self.declare_parameter(
            name="pose_covariance_position",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Position covariance for published pose (diagonal)",
            ),
        )

        self.declare_parameter(
            name="pose_covariance_orientation",
            value=0.05,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Orientation covariance for published pose (diagonal)",
            ),
        )

        # TF timeout
        self.declare_parameter(
            name="tf_timeout_seconds",
            value=0.1,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Timeout for TF lookups in seconds",
            ),
        )

        # Bootstrap pose parameters
        self.declare_parameter(
            name="publish_bootstrap_pose",
            value=True,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Publish origin pose when no boards known and TF fails (helps EKF bootstrap)",
            ),
        )

        self.declare_parameter(
            name="bootstrap_pose_rate",
            value=1.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Rate (Hz) to publish bootstrap pose when needed",
            ),
        )

        # Get parameter values
        self.aruco_topic = self.get_parameter("aruco_topic").value
        self.output_topic = self.get_parameter("output_topic").value

        self.map_frame = self.get_parameter("map_frame").value
        self.base_link_frame = self.get_parameter("base_link_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value

        self.min_samples_for_validation = self.get_parameter(
            "min_samples_for_validation"
        ).value
        self.validation_window_seconds = self.get_parameter(
            "validation_window_seconds"
        ).value
        self.max_position_std_meters = self.get_parameter(
            "max_position_std_meters"
        ).value
        self.candidate_timeout_seconds = self.get_parameter(
            "candidate_timeout_seconds"
        ).value
        self.candidate_cleanup_interval = self.get_parameter(
            "candidate_cleanup_interval"
        ).value

        self.pose_covariance_position = self.get_parameter(
            "pose_covariance_position"
        ).value
        self.pose_covariance_orientation = self.get_parameter(
            "pose_covariance_orientation"
        ).value

        self.tf_timeout_seconds = self.get_parameter("tf_timeout_seconds").value

        self.publish_bootstrap_pose = self.get_parameter("publish_bootstrap_pose").value
        self.bootstrap_pose_rate = self.get_parameter("bootstrap_pose_rate").value

    def aruco_callback(self, msg: ArucoBoard):
        """Process detected ArUco board"""
        board_id = msg.board_id

        # Check if this is a known board (lock only for the check)
        with self.lock:
            is_known = board_id in self.known_boards
        
        if is_known:
            # Use this board to estimate robot pose
            self.estimate_robot_pose(msg)
        else:
            # Process as candidate for validation
            self.process_candidate_board(msg)

    def process_candidate_board(self, msg: ArucoBoard):
        """Process a board that's being validated"""
        board_id = msg.board_id

        try:
            # Get the board position in map frame
            board_pose_map = self.transform_board_to_map(msg)
            if board_pose_map is None:
                return

            # Create sample
            sample = BoardSample(
                timestamp=Time.from_msg(msg.header.stamp),
                position=np.array(
                    [
                        board_pose_map.position.x,
                        board_pose_map.position.y,
                        board_pose_map.position.z,
                    ]
                ),
                orientation=np.array(
                    [
                        board_pose_map.orientation.x,
                        board_pose_map.orientation.y,
                        board_pose_map.orientation.z,
                        board_pose_map.orientation.w,
                    ]
                ),
            )
            
            # self.get_logger().info(f"Board {board_id} sample in map frame:")

            # Add to candidate (lock only for dictionary access)
            with self.lock:
                if board_id not in self.candidate_boards:
                    self.get_logger().info(f"Creating new candidate board entry: {board_id}")
                    self.candidate_boards[board_id] = BoardCandidate(board_id=board_id)
                    self.get_logger().info(f"New candidate board detected: {board_id}")

                self.candidate_boards[board_id].add_sample(sample)

            # Check if we can validate this board (outside lock)
            self.try_validate_board(board_id)

            # Publish candidate marker (red)
            self.publish_candidate_marker(board_id, sample.position)

        except Exception as e:
            self.get_logger().error(f"Error processing candidate board {board_id}: {e}")

    def try_validate_board(self, board_id: str):
        """Check if a candidate board meets validation criteria"""
        # Get candidate (with lock)
        with self.lock:
            if board_id not in self.candidate_boards:
                return
            candidate = self.candidate_boards[board_id]
        
        current_time = self.get_clock().now()
        window = Duration(seconds=self.validation_window_seconds)

        # Get recent samples
        recent_samples = candidate.get_recent_samples(current_time, window)

        if len(recent_samples) < self.min_samples_for_validation:
            return

        # Compute statistics
        result = candidate.compute_statistics(recent_samples)
        if result[0] is None:
            return

        mean_pos, mean_orientation, max_std = result

        self.get_logger().info(
            f"Board {board_id} validation result: "
            f"Mean Position: [{mean_pos[0]:.3f}, {mean_pos[1]:.3f}, {mean_pos[2]:.3f}], "
            f"Mean Orientation: [{mean_orientation[0]:.3f}, {mean_orientation[1]:.3f}, {mean_orientation[2]:.3f}, {mean_orientation[3]:.3f}], "
            f"Max Std: {max_std:.4f}m"
        )

        # Check if variance is acceptable
        if max_std <= self.max_position_std_meters:
            # Promote to known board (with lock)
            with self.lock:
                known_board = KnownBoard(
                    board_id=board_id,
                    position=mean_pos,
                    orientation=mean_orientation,
                    num_observations=len(recent_samples),
                    last_seen=current_time,
                )

                self.known_boards[board_id] = known_board
                # Only delete if still exists (race condition check)
                if board_id in self.candidate_boards:
                    del self.candidate_boards[board_id]

            self.get_logger().info(
                f"Board {board_id} validated and added to known boards! "
                f"Position: [{mean_pos[0]:.3f}, {mean_pos[1]:.3f}, {mean_pos[2]:.3f}], "
                f"Max std: {max_std:.4f}m"
            )

            # Publish known board marker (green) - only once
            self.publish_known_board_marker(board_id, mean_pos)
        else:
            self.get_logger().info(
                f"Board {board_id}: {len(recent_samples)} samples, max_std={max_std:.4f}m "
                f"(threshold: {self.max_position_std_meters}m)"
            )

    def transform_board_to_map(self, msg: ArucoBoard) -> Optional[Pose]:
        """Transform board pose from camera frame to map frame"""
        try:
            # Board pose is in camera frame
            camera_frame = msg.header.frame_id

            # Create PoseStamped from board detection
            board_pose_camera = PoseStamped()
            board_pose_camera.header = msg.header
            board_pose_camera.pose = msg.pose

            # Look up transform from camera to map
            # Use Time() to get the latest available transform to avoid extrapolation errors
            timeout = Duration(seconds=self.tf_timeout_seconds)
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, 
                camera_frame, 
                # msg.header.stamp, 
                Time(),
                timeout
            )

            # Transform the pose using PoseStamped
            board_pose_map = tf2_geometry_msgs.do_transform_pose_stamped(
                board_pose_camera, transform
            )

            return board_pose_map.pose

        except LookupException as e:
            # If no known boards exist and bootstrap is enabled, publish origin pose
            if self.publish_bootstrap_pose and len(self.known_boards) == 0:
                self.try_publish_bootstrap_pose(msg.header.stamp)

            self.get_logger().warn(
                f"~TF lookup failed for board to map: {e}", throttle_duration_sec=1.0
            )
            return None
        except (ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup failed for board to map: {e}", throttle_duration_sec=1.0
            )
            return None
        except Exception as e:
            self.get_logger().error(f"Error transforming board to map: {e} \n{traceback.format_exc()}")
            return None

    def estimate_robot_pose(self, msg: ArucoBoard):
        """Estimate robot pose using a known board"""
        board_id = msg.board_id

        # Get known board (with lock)
        with self.lock:
            if board_id not in self.known_boards:
                return
            known_board = self.known_boards[board_id]

        try:
            # Board detection is in camera frame
            # Known board location is in map frame
            # We need to compute: robot_pose_map

            # Get transform from base_link to camera
            timeout = Duration(seconds=self.tf_timeout_seconds)
            transform_base_to_camera = self.tf_buffer.lookup_transform(
                self.base_link_frame,
                msg.header.frame_id,  # camera frame
                # msg.header.stamp,
                Time(),
                timeout,
            )

            # Transform board detection to base_link frame
            board_pose_camera = PoseStamped()
            board_pose_camera.header = msg.header
            board_pose_camera.pose = msg.pose

            board_pose_base = tf2_geometry_msgs.do_transform_pose_stamped(
                board_pose_camera, transform_base_to_camera
            )

            # Now we have:
            # - board_pose_base: where the board is relative to base_link
            # - known_board: where the board is in map frame
            # We need to find where base_link is in map frame

            # This requires inverse transform:
            # T_map_base = T_map_board * T_board_base
            # where T_board_base = inv(T_base_board)

            robot_pose_map = self.compute_robot_pose_from_board(
                board_pose_base.pose, known_board
            )

            if robot_pose_map:
                # Update known board stats (with lock)
                with self.lock:
                    known_board.num_observations += 1
                    known_board.last_seen = self.get_clock().now()

                # Publish pose estimate with current time for better EKF integration
                current_stamp = self.get_clock().now().to_msg()
                self.publish_pose_estimate(robot_pose_map, current_stamp, board_id)

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f"TF lookup failed for robot pose estimation: {e}",
                throttle_duration_sec=1.0,
            )
        except Exception as e:
            self.get_logger().error(f"Error estimating robot pose: {e} {traceback.format_exc()}")

    def compute_robot_pose_from_board(
        self, board_pose_base: Pose, known_board: KnownBoard
    ) -> Optional[Pose]:
        """
        Compute robot pose in map frame given:
        - board_pose_base: board position relative to robot base_link
        - known_board: known board position in map frame

        Uses scipy.spatial.transform.Rotation for robust quaternion/matrix operations.
        """
        try:
            # 1. Create transformation matrix T_base_board
            t_base_board = np.array(
                [
                    board_pose_base.position.x,
                    board_pose_base.position.y,
                    board_pose_base.position.z,
                ]
            )
            q_base_board = np.array(
                [
                    board_pose_base.orientation.x,
                    board_pose_base.orientation.y,
                    board_pose_base.orientation.z,
                    board_pose_base.orientation.w,
                ]
            )
            r_base_board = Rotation.from_quat(q_base_board)

            T_base_board = np.eye(4)
            T_base_board[:3, :3] = r_base_board.as_matrix()
            T_base_board[:3, 3] = t_base_board

            # 2. Create transformation matrix T_map_board
            r_map_board = Rotation.from_quat(known_board.orientation)
            T_map_board = np.eye(4)
            T_map_board[:3, :3] = r_map_board.as_matrix()
            T_map_board[:3, 3] = known_board.position

            # 3. Compute T_map_base = T_map_board * inv(T_base_board)
            # This gives us the robot's pose in the map frame
            T_map_base = T_map_board @ np.linalg.inv(T_base_board)

            # 4. Extract pose from transformation matrix
            robot_pose = Pose()
            robot_pose.position.x = float(T_map_base[0, 3])
            robot_pose.position.y = float(T_map_base[1, 3])
            robot_pose.position.z = float(T_map_base[2, 3])

            # Extract quaternion from rotation matrix
            final_rot = Rotation.from_matrix(T_map_base[:3, :3])
            q_final = final_rot.as_quat()

            robot_pose.orientation.x = float(q_final[0])
            robot_pose.orientation.y = float(q_final[1])
            robot_pose.orientation.z = float(q_final[2])
            robot_pose.orientation.w = float(q_final[3])

            return robot_pose

        except Exception as e:
            self.get_logger().error(f"Error computing robot pose from board: {e}")
            return None

    def publish_pose_estimate(self, pose: Pose, stamp, board_id: str):
        """Publish robot pose estimate"""
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose = pose

        # Set covariance (6x6 matrix, flattened)
        # Order: x, y, z, rotation about X, rotation about Y, rotation about Z
        covariance = np.zeros(36)
        covariance[0] = self.pose_covariance_position  # x
        covariance[7] = self.pose_covariance_position  # y
        covariance[14] = self.pose_covariance_position  # z
        covariance[21] = self.pose_covariance_orientation  # rot x
        covariance[28] = self.pose_covariance_orientation  # rot y
        covariance[35] = self.pose_covariance_orientation  # rot z
        msg.pose.covariance = covariance.tolist()

        self.pose_pub.publish(msg)

        self.get_logger().debug(
            f"Published pose estimate from board {board_id}: "
            f"[{pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}]"
        )

    def try_publish_bootstrap_pose(self, stamp):
        """
        Publish an origin pose to help EKF bootstrap when no boards are known.
        Rate-limited to avoid spamming.
        """
        current_time = self.get_clock().now()

        # Rate limit bootstrap pose publishing
        if self.last_bootstrap_time is not None:
            time_since_last = (
                current_time - self.last_bootstrap_time
            ).nanoseconds / 1e9
            min_interval = 1.0 / self.bootstrap_pose_rate

            if time_since_last < min_interval:
                return

        # Create origin pose
        origin_pose = Pose()
        origin_pose.position.x = 0.0
        origin_pose.position.y = 0.0
        origin_pose.position.z = 0.0
        origin_pose.orientation.x = 0.0
        origin_pose.orientation.y = 0.0
        origin_pose.orientation.z = 0.0
        origin_pose.orientation.w = 1.0

        # Publish with high covariance to indicate uncertainty
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.pose.pose = origin_pose

        # High covariance for bootstrap (10x normal)
        covariance = np.zeros(36)
        covariance[0] = self.pose_covariance_position * 10.0  # x
        covariance[7] = self.pose_covariance_position * 10.0  # y
        covariance[14] = self.pose_covariance_position * 10.0  # z
        covariance[21] = self.pose_covariance_orientation * 10.0  # rot x
        covariance[28] = self.pose_covariance_orientation * 10.0  # rot y
        covariance[35] = self.pose_covariance_orientation * 10.0  # rot z
        msg.pose.covariance = covariance.tolist()

        self.pose_pub.publish(msg)

        self.last_bootstrap_time = current_time
        self.bootstrap_published_count += 1

        if self.bootstrap_published_count <= 5:  # Log first few times
            self.get_logger().info(
                "Published bootstrap origin pose to help EKF initialize "
                f"(count: {self.bootstrap_published_count})"
            )
        elif self.bootstrap_published_count % 10 == 0:  # Then every 10th time
            self.get_logger().debug(
                f"Bootstrap pose published {self.bootstrap_published_count} times"
            )

    def publish_candidate_marker(self, board_id: str, position: np.ndarray):
        """Publish a red sphere marker for a candidate board"""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "candidate_boards"
        marker.id = hash(board_id) % 10000  # Unique ID from board_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0

        # Size
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Color - Red for candidates
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8  # Semi-transparent

        # Lifetime - will disappear if not updated
        marker.lifetime.sec = 2
        marker.lifetime.nanosec = 0

        # Publish as MarkerArray
        marker_array = MarkerArray()
        marker_array.markers = [marker]
        self.marker_pub.publish(marker_array)

    def publish_known_board_marker(self, board_id: str, position: np.ndarray):
        """Publish a green sphere marker for a validated known board"""
        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "known_boards"
        marker.id = hash(board_id) % 10000  # Unique ID from board_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0

        # Size
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        # Color - Green for known boards
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Lifetime - persistent (0 means forever)
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0

        # Publish as MarkerArray
        marker_array = MarkerArray()
        marker_array.markers = [marker]
        self.marker_pub.publish(marker_array)

        self.get_logger().info(f"Published green marker for validated board {board_id}")

    def cleanup_old_candidates(self):
        """Remove candidate boards that haven't been seen recently"""
        current_time = self.get_clock().now()
        timeout = Duration(seconds=self.candidate_timeout_seconds)

        with self.lock:
            boards_to_remove = []

            for board_id, candidate in self.candidate_boards.items():
                if not candidate.samples:
                    continue

                last_sample_time = candidate.samples[-1].timestamp
                if (current_time - last_sample_time) > timeout:
                    boards_to_remove.append(board_id)

            for board_id in boards_to_remove:
                del self.candidate_boards[board_id]
                self.get_logger().info(f"Removed stale candidate board: {board_id}")

    def get_status_summary(self) -> str:
        """Get a summary of the current state"""
        with self.lock:
            num_known = len(self.known_boards)
            num_candidates = len(self.candidate_boards)

            summary = f"Known boards: {num_known}, Candidate boards: {num_candidates}"

            if self.known_boards:
                summary += "\nKnown boards:"
                for board_id, board in self.known_boards.items():
                    summary += (
                        f"\n  - {board_id}: {board.num_observations} observations"
                    )

            return summary


def main(args=None):
    rclpy.init(args=args)
    node = ArucoBoardLocalizerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Print final status
        node.get_logger().info("Shutting down...")
        node.get_logger().info(node.get_status_summary())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
