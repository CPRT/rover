#!/usr/bin/env python3
"""
ArUco Board Detector Node

This node detects pairs of ArUco markers configured as boards and estimates
their pose using cv2.aruco.estimatePoseBoard. It publishes the board poses
transformed into the ROS 2 Standard Body Frame (X-Forward, Y-Left, Z-Up).

Subscriptions:
   /camera/image_raw (sensor_msgs.msg.Image)
   /camera/camera_info (sensor_msgs.msg.CameraInfo)

Published Topics:
    /aruco_boards (interfaces.msg.ArucoBoard)
       Pose of detected ArUco boards
    
    /aruco_debug (sensor_msgs.msg.Image)
       Debug image with detected boards and axes drawn

Parameters:
    image_topic - image topic to subscribe to
    camera_info_topic - camera info topic to subscribe to
    camera_frame - camera optical frame to use
    aruco_dictionary_id - dictionary that was used to generate markers
    board_configs - list of board configuration file paths
    debug_image_topic - topic to publish debug image

Author: GitHub Copilot (Corrected)
Version: 2025-12-06
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import cv2
import numpy as np
import yaml
import os
from pathlib import Path
from scipy.spatial.transform import Rotation

# Import OpenCV compatibility layer
from . import cv2_utils

# ROS2 message imports
from sensor_msgs.msg import CameraInfo, Image
from interfaces.msg import ArucoBoard
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from tf_transformations import quaternion_from_matrix


# ArUco dictionary mapping
ARUCO_DICT = {
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
    "DICT_4X4_250": cv2.aruco.DICT_4X4_250,
    "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
    "DICT_5X5_50": cv2.aruco.DICT_5X5_50,
    "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
    "DICT_6X6_50": cv2.aruco.DICT_6X6_50,
    "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
    "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
    "DICT_7X7_50": cv2.aruco.DICT_7X7_50,
    "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
    "DICT_7X7_250": cv2.aruco.DICT_7X7_250,
    "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
    "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


class ArucoBoardConfig:
    """Class to hold ArUco board configuration"""

    def __init__(self, board_id, marker_ids, corners, tag_size):
        self.board_id = board_id
        self.marker_ids = marker_ids
        self.corners = corners
        self.tag_size = tag_size

        self.obj_points = []
        self.ids = []

        for marker_id, marker_corners in zip(marker_ids, corners):
            self.ids.append(marker_id)
            corner_array = np.array(marker_corners, dtype=np.float32)
            self.obj_points.append(corner_array)

        self.obj_points = np.array(self.obj_points, dtype=np.float32)
        self.ids = np.array(self.ids, dtype=np.int32)

        # Note: Board uses a placeholder dictionary during initialization.
        # The actual dictionary from the detector node will be used during detection.
        dictionary = cv2_utils.get_aruco_dictionary(cv2.aruco.DICT_6X6_250)
        self.board = cv2_utils.create_aruco_board(
            self.obj_points,
            dictionary,
            self.ids,
        )


class ArucoBoardDetectorNode(Node):
    def __init__(self):
        super().__init__("aruco_board_detector_node")

        self.initialize_parameters()

        try:
            dictionary_id = ARUCO_DICT[self.aruco_dictionary_id]
        except KeyError:
            self.get_logger().error(
                f"ArUco dictionary '{self.aruco_dictionary_id}' not supported. "
                f"Valid options: {list(ARUCO_DICT.keys())}"
            )
            raise

        # Use compatibility layer for ArUco dictionary and parameters
        self.aruco_dictionary = cv2_utils.get_aruco_dictionary(dictionary_id)
        self.aruco_parameters = cv2_utils.create_detector_parameters()

        self.boards = []
        self.load_board_configs()

        self.info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.info_callback,
            qos_profile_sensor_data,
        )

        self.image_sub = self.create_subscription(
            Image, self.image_topic, self.image_callback, qos_profile_sensor_data
        )

        self.board_pub = self.create_publisher(ArucoBoard, self.board_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 10)
        self.marker_viz_pub = self.create_publisher(
            MarkerArray, self.marker_visualization_topic, 10
        )

        self.info_msg = None
        self.intrinsic_mat = None
        self.distortion = None
        self.bridge = CvBridge()

        # Log OpenCV version information
        api_info = cv2_utils.get_api_info()
        self.get_logger().info(
            f"ArUco Board Detector Node initialized with {len(self.boards)} boards"
        )
        self.get_logger().info(
            f"OpenCV Version: {api_info['opencv_version']} "
            f"(using {'new' if api_info['using_new_api'] else 'legacy'} ArUco API)"
        )

    def initialize_parameters(self):
        """Declare and read ROS2 parameters"""

        self.declare_parameter(
            name="image_topic",
            value="/camera/color/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Image topic to subscribe to",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera info topic to subscribe to",
            ),
        )

        self.declare_parameter(
            name="camera_frame",
            value="camera_color_frame",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera optical frame ID",
            ),
        )

        self.declare_parameter(
            name="aruco_dictionary_id",
            value="DICT_6X6_250",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING, description="ArUco dictionary ID"
            ),
        )

        self.declare_parameter(
            name="board_config_dir",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Directory containing board configuration YAML files",
            ),
        )

        self.declare_parameter(
            name="board_topic",
            value="/aruco_boards",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish detected board poses",
            ),
        )

        self.declare_parameter(
            name="debug_image_topic",
            value="/aruco_debug",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish debug images",
            ),
        )

        self.declare_parameter(
            name="marker_visualization_topic",
            value="/aruco_markers_viz",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish marker visualizations",
            ),
        )

        self.declare_parameter(
            name="marker_size",
            value=0.05,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of visualization markers in meters",
            ),
        )

        self.declare_parameter(
            name="marker_thickness",
            value=0.005,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Thickness (depth) of ArUco tag visualization in meters",
            ),
        )

        self.declare_parameter(
            name="arrow_length",
            value=0.08,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Length of arrow showing tag orientation in meters",
            ),
        )

        self.declare_parameter(
            name="min_markers_for_board",
            value=2,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Minimum number of markers required to detect a board",
            ),
        )

        # Get parameter values
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.aruco_dictionary_id = self.get_parameter("aruco_dictionary_id").value
        self.board_config_dir = self.get_parameter("board_config_dir").value
        self.board_topic = self.get_parameter("board_topic").value
        self.debug_image_topic = self.get_parameter("debug_image_topic").value
        self.marker_visualization_topic = self.get_parameter(
            "marker_visualization_topic"
        ).value
        self.marker_size = self.get_parameter("marker_size").value
        self.marker_thickness = self.get_parameter("marker_thickness").value
        self.arrow_length = self.get_parameter("arrow_length").value
        self.min_markers_for_board = self.get_parameter("min_markers_for_board").value

    def load_board_configs(self):
        if not self.board_config_dir or not os.path.exists(self.board_config_dir):
            self.get_logger().warn(
                f"Board config directory '{self.board_config_dir}' does not exist. "
                "No boards will be loaded."
            )
            return

        config_files = list(Path(self.board_config_dir).glob("*.yaml"))
        self.get_logger().info(f"Found {len(config_files)} board config files")

        for config_file in config_files:
            try:
                with open(config_file, "r") as f:
                    config = yaml.safe_load(f)
                board_id = config_file.stem
                marker_ids = config.get("ids", [])
                corners = config.get("corners", [])
                tag_size = config.get("tag_size", 0.1)

                if not marker_ids or not corners:
                    self.get_logger().warn(
                        f"Invalid config in {config_file.name}: missing ids or corners"
                    )
                    continue

                board_config = ArucoBoardConfig(board_id, marker_ids, corners, tag_size)
                self.boards.append(board_config)
                self.get_logger().info(f"Loaded board '{board_id}'")
            except Exception as e:
                self.get_logger().error(f"Failed to load {config_file.name}: {e}")

    def info_callback(self, info_msg):
        self.info_msg = info_msg
        self.intrinsic_mat = np.reshape(np.array(self.info_msg.k), (3, 3))
        self.distortion = np.array(self.info_msg.d)
        self.get_logger().info("Camera info received.")
        self.destroy_subscription(self.info_sub)

    def image_callback(self, img_msg: Image):
        if self.info_msg is None:
            self.get_logger().warn(
                "Camera info not received yet, skipping image",
                throttle_duration_sec=1.0,
            )
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="rgb8")
            # Use compatibility layer for marker detection
            corners, marker_ids, rejected = cv2_utils.detect_markers(
                cv_image, self.aruco_dictionary, self.aruco_parameters
            )

            debug_image = cv_image.copy()
            marker_array = MarkerArray()
            marker_id_counter = 0

            if marker_ids is not None and len(marker_ids) > 0:
                # Use compatibility layer for drawing markers
                cv2_utils.draw_detected_markers(debug_image, corners, marker_ids)
                marker_ids = marker_ids.flatten()

                for board_config in self.boards:
                    detected_corners = []
                    detected_ids = []

                    # Match markers to board
                    for i, marker_id in enumerate(marker_ids):
                        if marker_id in board_config.marker_ids:
                            detected_corners.append(corners[i])
                            detected_ids.append(marker_id)

                    if len(detected_ids) >= self.min_markers_for_board:
                        success, rvec, tvec = self.estimate_board_pose(
                            board_config, detected_corners, detected_ids
                        )

                        if success:
                            ros_pose_data = self.publish_board_pose(
                                board_config, rvec, tvec, img_msg.header, detected_ids
                            )

                            # Use compatibility layer for drawing axis
                            cv2_utils.draw_axis(
                                debug_image,
                                self.intrinsic_mat,
                                self.distortion,
                                rvec,
                                tvec,
                                board_config.tag_size,
                            )

                            if ros_pose_data:
                                markers = self.create_tag_center_markers(
                                    board_config,
                                    ros_pose_data,
                                    img_msg.header,
                                    marker_id_counter,
                                )
                                marker_array.markers.extend(markers)
                                marker_id_counter += len(markers)

            # Publish Debug
            debug_msg = self.bridge.cv2_to_imgmsg(debug_image, encoding="rgb8")
            debug_msg.header = img_msg.header
            self.debug_pub.publish(debug_msg)
            self.marker_viz_pub.publish(marker_array)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def estimate_board_pose(self, board_config, detected_corners, detected_ids):
        try:
            detected_corners_array = [c for c in detected_corners]
            detected_ids_array = np.array(detected_ids, dtype=np.int32).reshape(-1, 1)

            # Use compatibility layer for pose estimation
            num_markers, rvec, tvec = cv2_utils.estimate_pose_board(
                detected_corners_array,
                detected_ids_array,
                board_config.board,
                self.intrinsic_mat,
                self.distortion,
            )

            if num_markers > 0 and rvec is not None and tvec is not None:
                # Validate the pose to reject flipped/ambiguous solutions
                if not self.validate_pose(tvec, rvec):
                    self.get_logger().debug(
                        f"Rejected invalid pose for board {board_config.board_id}",
                        throttle_duration_sec=1.0,
                    )
                    return False, None, None
                return True, rvec, tvec
            return False, None, None
        except Exception as e:
            self.get_logger().error(f"Error estimating pose: {e}")
            return False, None, None

    def validate_pose(self, tvec, rvec):
        """
        Validate the estimated pose to reject ambiguous or flipped solutions.

        ArUco pose estimation can sometimes return a 180-degree flipped solution
        where the board appears behind the camera or with impossible orientation.

        Validation checks:
        1. Board must be in front of camera (positive Z in OpenCV camera frame)
        2. Distance should be reasonable (not too close, not too far)
        3. Board normal should generally face the camera
        """
        try:
            # Check 1: Board must be in front of camera (Z > 0 in OpenCV convention)
            z_distance = float(tvec[2][0])
            if z_distance <= 0:
                return False

            # Check 2: Reasonable distance bounds (adjust based on your use case)
            # Reject if too close (< 0.2m) or too far (> 20m)
            if z_distance < 0.2 or z_distance > 20.0:
                return False

            # Check 3: Board normal should roughly face the camera
            # Convert rotation vector to rotation matrix
            rot_mat, _ = cv2.Rodrigues(rvec)

            # In OpenCV convention, the board's Z-axis (normal) is the 3rd column
            board_normal = rot_mat[:, 2]

            # Vector from camera to board
            camera_to_board = tvec.flatten() / np.linalg.norm(tvec)

            # The board normal should be roughly opposite to the camera-to-board vector
            # (i.e., the board should be facing the camera)
            # dot product should be negative (angles between 90-270 degrees)
            dot_product = np.dot(board_normal, camera_to_board)

            # Allow some tolerance - board doesn't need to be perfectly perpendicular
            # but should be generally facing the camera (dot < 0.5 means angle > 60 degrees)
            if dot_product > 0.5:
                return False

            return True

        except Exception as e:
            self.get_logger().warn(f"Error validating pose: {e}")
            return False

    def publish_board_pose(self, board_config, rvec, tvec, header, detected_ids):
        """
        Transforms from OpenCV convention to ROS2 Standard Body convention.
        OpenCV: X-right, Y-down, Z-forward (Optical)
        ROS2 Body: X-forward, Y-left, Z-up
        """
        try:
            msg = ArucoBoard()
            # Note: If transforming to ROS Body convention, frame_id should ideally
            # match a body-oriented frame (e.g., camera_link), not camera_color_optical_frame.
            msg.header.frame_id = self.camera_frame or header.frame_id
            msg.header.stamp = header.stamp
            msg.board_id = board_config.board_id
            msg.marker_ids = [int(x) for x in detected_ids]

            rot_mat_cv, _ = cv2.Rodrigues(rvec)

            # --- FIXED TRANSFORMATION MATRIX ---
            # Input (CV): X(Right), Y(Down), Z(Forward)
            # Output (ROS): X(Forward), Y(Left), Z(Up)
            # Logic:
            # ROS_X = CV_Z
            # ROS_Y = -CV_X
            # ROS_Z = -CV_Y
            T_cv_to_ros = np.array(
                [[0, 0, 1], [-1, 0, 0], [0, -1, 0]], dtype=np.float32
            )

            # 1. Transform Position
            tvec_ros = T_cv_to_ros @ tvec

            # 2. Transform Rotation
            # We apply T to the rotation result from CV to align it with ROS axes
            rot_mat_ros = T_cv_to_ros @ rot_mat_cv

            # Convert to Quaternion
            rot_matrix_4x4 = np.eye(4, dtype=np.float32)
            rot_matrix_4x4[0:3, 0:3] = rot_mat_ros
            quaternion = quaternion_from_matrix(rot_matrix_4x4)
            quaternion = quaternion / np.linalg.norm(quaternion)

            msg.pose.position.x = float(tvec_ros[0][0])
            msg.pose.position.y = float(tvec_ros[1][0])
            msg.pose.position.z = float(tvec_ros[2][0])
            msg.pose.orientation.x = float(quaternion[0])
            msg.pose.orientation.y = float(quaternion[1])
            msg.pose.orientation.z = float(quaternion[2])
            msg.pose.orientation.w = float(quaternion[3])

            # Transform corners
            for marker_corners_3d in board_config.obj_points:
                for corner_3d in marker_corners_3d:
                    # Point in CV Camera Frame
                    corner_camera_cv = rot_mat_cv @ corner_3d.reshape(3, 1) + tvec
                    # Point in ROS Body Frame
                    corner_camera_ros = T_cv_to_ros @ corner_camera_cv

                    point = Point()
                    point.x = float(corner_camera_ros[0][0])
                    point.y = float(corner_camera_ros[1][0])
                    point.z = float(corner_camera_ros[2][0])
                    msg.corners_3d.append(point)

            self.board_pub.publish(msg)

            return {
                "rot_mat_ros": rot_mat_ros,
                "tvec_ros": tvec_ros,
                "T_cv_to_ros": T_cv_to_ros,
            }

        except Exception as e:
            self.get_logger().error(f"Error publishing board pose: {e}")
            return None

    def create_tag_center_markers(
        self, board_config, ros_pose_data, header, marker_id_start
    ):
        """
        Creates markers in the ROS2 Body Frame (X-Fwd) using the result from publish_board_pose.
        """
        markers = []
        if ros_pose_data is None:
            return markers

        try:
            rot_mat_ros = ros_pose_data["rot_mat_ros"]
            tvec_ros = ros_pose_data["tvec_ros"]

            for i, (marker_id, marker_corners_3d) in enumerate(
                zip(board_config.marker_ids, board_config.obj_points)
            ):
                # 1. Get Board-Local Center
                center_local = np.mean(marker_corners_3d, axis=0).reshape(3, 1)

                # 2. Get Board-Local Orientation (CV convention)
                c0 = marker_corners_3d[0].reshape(3, 1)
                c1 = marker_corners_3d[1].reshape(3, 1)
                c3 = marker_corners_3d[3].reshape(3, 1)

                x_axis = c1 - c0
                x_axis /= np.linalg.norm(x_axis)
                y_axis = c3 - c0
                y_axis /= np.linalg.norm(y_axis)
                z_axis = np.cross(x_axis.flatten(), y_axis.flatten()).reshape(3, 1)
                z_axis /= np.linalg.norm(z_axis)

                # Orientation of the specific marker relative to the board
                rot_local = np.column_stack([x_axis, y_axis, z_axis])

                # 3. Transform to ROS Body Frame
                # Position: Apply the Board Pose (which is already in ROS frame)
                # Note: We need to be careful here.
                # rot_mat_ros takes a vector from BOARD frame and puts it in BODY frame.
                center_ros = rot_mat_ros @ center_local + tvec_ros

                # Rotation: The board pose rotation applied to the marker local rotation
                rot_final = rot_mat_ros @ rot_local

                # Convert to quaternion
                transform_4x4 = np.eye(4)
                transform_4x4[0:3, 0:3] = rot_final
                quat = quaternion_from_matrix(transform_4x4)
                quat /= np.linalg.norm(quat)

                # --- Cube Marker ---
                tag_w = np.linalg.norm(c1 - c0)
                tag_h = np.linalg.norm(c3 - c0)

                cube = Marker()
                cube.header.frame_id = self.camera_frame or header.frame_id
                cube.header.stamp = header.stamp
                cube.ns = f"{board_config.board_id}_tags"
                cube.id = marker_id_start + i * 3
                cube.type = Marker.CUBE
                cube.action = Marker.ADD
                cube.pose.position.x = float(center_ros[0][0])
                cube.pose.position.y = float(center_ros[1][0])
                cube.pose.position.z = float(center_ros[2][0])
                cube.pose.orientation.x = float(quat[0])
                cube.pose.orientation.y = float(quat[1])
                cube.pose.orientation.z = float(quat[2])
                cube.pose.orientation.w = float(quat[3])
                cube.scale.x = float(tag_w)
                cube.scale.y = float(tag_h)
                cube.scale.z = self.marker_thickness

                if i == 0:
                    color = (0.0, 1.0, 0.0)
                elif i == 1:
                    color = (0.0, 0.5, 1.0)
                else:
                    color = (1.0, 1.0, 0.0)

                cube.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=0.6)
                markers.append(cube)

                # --- Arrow Marker ---
                # RViz Arrow points along its local X-axis by default.
                # We want it to point along the marker's Z-axis (normal).
                # In the marker's frame: X is right edge, Y is bottom edge, Z is normal (out).
                # We need to rotate so that Arrow's X points along Marker's Z.
                # This is a +90 degree rotation around the marker's Y-axis.
                arrow = Marker()
                arrow.header = cube.header
                arrow.ns = f"{board_config.board_id}_arrows"
                arrow.id = cube.id + 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.pose.position = cube.pose.position

                # Rotation logic: Rotate +90 degrees around Y to align Arrow-X with Marker-Z
                rot_y_90 = Rotation.from_euler("y", 90, degrees=True)
                current_rot = Rotation.from_quat(quat)
                final_arrow_rot = current_rot * rot_y_90
                arrow_quat = final_arrow_rot.as_quat()

                arrow.pose.orientation.x = float(arrow_quat[0])
                arrow.pose.orientation.y = float(arrow_quat[1])
                arrow.pose.orientation.z = float(arrow_quat[2])
                arrow.pose.orientation.w = float(arrow_quat[3])

                arrow.scale.x = self.arrow_length
                arrow.scale.y = self.arrow_length * 0.15
                arrow.scale.z = self.arrow_length * 0.2
                arrow.color = ColorRGBA(
                    r=min(1.0, color[0] + 0.3),
                    g=min(1.0, color[1] + 0.3),
                    b=min(1.0, color[2] + 0.3),
                    a=0.9,
                )
                markers.append(arrow)

                # --- Text Marker ---
                text = Marker()
                text.header = cube.header
                text.ns = f"{board_config.board_id}_ids"
                text.id = cube.id + 2
                text.type = Marker.TEXT_VIEW_FACING
                text.action = Marker.ADD

                # Offset text along the tag normal (Z in marker frame)
                # rot_final gives us the axes of the marker in ROS frame.
                # The 3rd column of rot_final is the Z-axis (normal).
                z_axis_ros = rot_final[:, 2].reshape(3, 1)
                text_offset = (self.arrow_length + 0.02) * z_axis_ros

                text.pose.position.x = float(center_ros[0][0] + text_offset[0][0])
                text.pose.position.y = float(center_ros[1][0] + text_offset[1][0])
                text.pose.position.z = float(center_ros[2][0] + text_offset[2][0])
                text.pose.orientation.w = 1.0
                text.text = f"ID:{marker_id}"
                text.scale.z = max(0.02, float(tag_w) * 0.5)
                text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
                markers.append(text)

        except Exception as e:
            self.get_logger().error(f"Error creating markers: {e}")

        return markers


def main(args=None):
    rclpy.init(args=args)
    node = ArucoBoardDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Allow clean shutdown on Ctrl+C without printing a stack trace
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
