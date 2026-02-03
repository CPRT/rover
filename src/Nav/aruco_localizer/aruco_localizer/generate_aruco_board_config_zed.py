#!/usr/bin/env python3
"""
Generate ArUco Board Config Node (ZED Camera)

This node subscribes to the ZED camera topic, collects images of an ArUco board
with two tags at different angles, and generates a precise 3D configuration file
for the board layout.

Subscribed Topics:
    /zed/zed_node/left/image_rect_color (sensor_msgs.msg.Image)
       Rectified color images from ZED camera

Parameters:
    camera_yaml_path - Path to camera calibration YAML file
    output_board_yaml - Path to output board configuration YAML file
    tag_size - Size of the printed tag in meters
    id_anchor - The ID you want to be at (0,0,0)
    id_target - The ID you want to calculate the position of
    capture_interval - Seconds between captures
    image_topic - Topic to subscribe to for images (default: /zed/zed_node/left/image_rect_color)


ros2 run aruco_localizer generate_aruco_board_config_zed --ros-args \
  -p image_topic:=/zed/zed_node/left/image_rect_color \
  -p tag_size:=0.1995 \
  -p id_anchor:=6 \
  -p id_target:=7

Version: 2026-02-03
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import cv2
import numpy as np
import yaml
import time
import sys
from scipy.spatial.transform import Rotation as R

# ROS2 message imports
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class GenerateArucoBoardConfigZed(Node):
    def __init__(self):
        super().__init__("generate_aruco_board_config_zed")

        # Declare parameters
        self.declare_parameter(
            "camera_yaml_path",
            "camera-0.yaml",
            ParameterDescriptor(
                name="camera_yaml_path",
                type=ParameterType.PARAMETER_STRING,
                description="Path to camera calibration YAML file",
            ),
        )
        self.declare_parameter(
            "output_board_yaml",
            "camera-0_ArucoBoard_Tag6-Tag7.yaml",
            ParameterDescriptor(
                name="output_board_yaml",
                type=ParameterType.PARAMETER_STRING,
                description="Path to output board configuration YAML file",
            ),
        )
        self.declare_parameter(
            "tag_size",
            0.1995,
            ParameterDescriptor(
                name="tag_size",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Size of the printed tag in meters",
            ),
        )
        self.declare_parameter(
            "id_anchor",
            6,
            ParameterDescriptor(
                name="id_anchor",
                type=ParameterType.PARAMETER_INTEGER,
                description="The ID you want to be at (0,0,0)",
            ),
        )
        self.declare_parameter(
            "id_target",
            7,
            ParameterDescriptor(
                name="id_target",
                type=ParameterType.PARAMETER_INTEGER,
                description="The ID you want to calculate the position of",
            ),
        )
        self.declare_parameter(
            "capture_interval",
            0.2,
            ParameterDescriptor(
                name="capture_interval",
                type=ParameterType.PARAMETER_DOUBLE,
                description="Seconds between captures",
            ),
        )
        self.declare_parameter(
            "image_topic",
            "/zed/zed_node/left/image_rect_color",
            ParameterDescriptor(
                name="image_topic",
                type=ParameterType.PARAMETER_STRING,
                description="Topic to subscribe to for images",
            ),
        )

        # Get parameters
        self.camera_yaml_path = (
            self.get_parameter("camera_yaml_path").get_parameter_value().string_value
        )
        self.output_board_yaml = (
            self.get_parameter("output_board_yaml").get_parameter_value().string_value
        )
        self.tag_size = (
            self.get_parameter("tag_size").get_parameter_value().double_value
        )
        self.id_anchor = (
            self.get_parameter("id_anchor").get_parameter_value().integer_value
        )
        self.id_target = (
            self.get_parameter("id_target").get_parameter_value().integer_value
        )
        self.capture_interval = (
            self.get_parameter("capture_interval").get_parameter_value().double_value
        )
        self.image_topic = (
            self.get_parameter("image_topic").get_parameter_value().string_value
        )

        # Initialize CV Bridge
        self.bridge = CvBridge()

        # Load camera intrinsics
        try:
            self.mtx, self.dist = self.load_camera_info(self.camera_yaml_path)
            self.get_logger().info(
                f"Loaded camera parameters from {self.camera_yaml_path}"
            )
        except FileNotFoundError:
            self.get_logger().error(f"Could not find {self.camera_yaml_path}")
            sys.exit(1)

        # Setup ArUco detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # State variables
        self.captured_frames = []
        self.last_capture_time = time.time()
        self.current_frame = None
        self.processing = False

        # Subscribe to image topic
        self.image_subscription = self.create_subscription(
            Image, self.image_topic, self.image_callback, 10
        )

        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info(f" PHASE 1: DATA COLLECTION")
        self.get_logger().info(
            f" Point camera at BOTH tags ({self.id_anchor} and {self.id_target})."
        )
        self.get_logger().info(f" Capturing every {self.capture_interval}s when valid.")
        self.get_logger().info(f" Press 'ENTER' in the window to finish and process.")
        self.get_logger().info("=" * 50 + "\n")

    def load_camera_info(self, yaml_path):
        """Parses the ROS-style camera YAML file."""
        with open(yaml_path, "r") as f:
            data = yaml.safe_load(f)

        # Extract Camera Matrix
        camera_matrix = np.array(
            data["camera_matrix"]["data"], dtype=np.float64
        ).reshape(3, 3)

        # Extract Distortion Coefficients
        dist_coeffs = np.array(
            data["distortion_coefficients"]["data"], dtype=np.float64
        )

        # Setup for 8 coefficients (Rational Polynomial) if needed
        if dist_coeffs.size > 5:
            dist_coeffs = dist_coeffs.reshape(1, -1)

        return camera_matrix, dist_coeffs

    def image_callback(self, msg):
        """Callback for image messages from ZED camera."""
        if self.processing:
            return

        try:
            # Convert ROS Image message to OpenCV image
            # ZED publishes in bgra8 encoding
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.current_frame = cv_image

            # Detect markers
            corners, ids, rejected = self.detector.detectMarkers(cv_image)
            display_frame = cv_image.copy()

            valid_frame = False
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
                flat_ids = ids.flatten()
                if self.id_anchor in flat_ids and self.id_target in flat_ids:
                    valid_frame = True

            current_time = time.time()

            # Capture logic
            if valid_frame and (
                current_time - self.last_capture_time > self.capture_interval
            ):
                self.captured_frames.append(cv_image.copy())
                self.last_capture_time = current_time
                self.get_logger().info(
                    f"Captured Frame #{len(self.captured_frames)}",
                    throttle_duration_sec=0.5,
                )

                # Flash effect
                cv2.rectangle(display_frame, (0, 0), (1280, 720), (0, 255, 0), 10)

            # Display
            cv2.imshow("Calibration Capture (Press Enter to Stop)", display_frame)
            key = cv2.waitKey(1)

            if key == 13:  # Enter key
                self.process_captured_frames()
            elif key == 27:  # ESC key
                self.get_logger().info("Cancelled.")
                cv2.destroyAllWindows()
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f"Error in image callback: {str(e)}")

    def process_captured_frames(self):
        """Process all captured frames and generate board configuration."""
        self.processing = True
        cv2.destroyAllWindows()

        if len(self.captured_frames) == 0:
            self.get_logger().warn("No frames captured. Exiting.")
            rclpy.shutdown()
            return

        self.get_logger().info(
            f"\n\nPHASE 2: PROCESSING {len(self.captured_frames)} FRAMES..."
        )

        relative_translations = []
        relative_rotations = []  # Quaternions

        for i, frame in enumerate(self.captured_frames):
            corners, ids, _ = self.detector.detectMarkers(frame)

            # We know both IDs exist because we filtered in Phase 1, but check again to be safe
            idx_anchor = np.where(ids == self.id_anchor)[0][0]
            idx_target = np.where(ids == self.id_target)[0][0]

            # Solve PnP for Anchor (Tag A)
            _, rvec_a, tvec_a = cv2.solvePnP(
                np.array(
                    [
                        [-self.tag_size / 2, self.tag_size / 2, 0],
                        [self.tag_size / 2, self.tag_size / 2, 0],
                        [self.tag_size / 2, -self.tag_size / 2, 0],
                        [-self.tag_size / 2, -self.tag_size / 2, 0],
                    ],
                    dtype=np.float32,
                ),
                corners[idx_anchor],
                self.mtx,
                self.dist,
            )

            # Solve PnP for Target (Tag B)
            _, rvec_b, tvec_b = cv2.solvePnP(
                np.array(
                    [
                        [-self.tag_size / 2, self.tag_size / 2, 0],
                        [self.tag_size / 2, self.tag_size / 2, 0],
                        [self.tag_size / 2, -self.tag_size / 2, 0],
                        [-self.tag_size / 2, -self.tag_size / 2, 0],
                    ],
                    dtype=np.float32,
                ),
                corners[idx_target],
                self.mtx,
                self.dist,
            )

            # Convert to Matrix
            R_a, _ = cv2.Rodrigues(rvec_a)
            T_cam_a = np.eye(4)
            T_cam_a[:3, :3] = R_a
            T_cam_a[:3, 3] = tvec_a.flatten()

            R_b, _ = cv2.Rodrigues(rvec_b)
            T_cam_b = np.eye(4)
            T_cam_b[:3, :3] = R_b
            T_cam_b[:3, 3] = tvec_b.flatten()

            # Calculate Transform from Anchor to Target
            # T_a_b = inv(T_cam_a) * T_cam_b
            T_a_b = np.linalg.inv(T_cam_a) @ T_cam_b

            relative_translations.append(T_a_b[:3, 3])
            relative_rotations.append(R.from_matrix(T_a_b[:3, :3]).as_quat())

        # Compute Statistics & Average
        trans_array = np.array(relative_translations)
        avg_translation = np.mean(trans_array, axis=0)
        std_translation = np.std(trans_array, axis=0)

        # Average Rotations (Mean of Quaternions)
        quat_array = np.array(relative_rotations)
        # Simple mean and normalize is sufficient for clustered data
        mean_quat = np.mean(quat_array, axis=0)
        mean_quat /= np.linalg.norm(mean_quat)

        avg_rot_matrix = R.from_quat(mean_quat).as_matrix()

        # Calculate Rotation Jitter (RMSE in degrees)
        angular_errors = []
        for q in quat_array:
            # Angle between mean_quat and current q
            # 2 * arccos(|<q1, q2>|)
            dot = abs(np.dot(q, mean_quat))
            if dot > 1.0:
                dot = 1.0
            angle = 2 * np.arccos(dot)
            angular_errors.append(np.degrees(angle))
        rmse_rotation = np.sqrt(np.mean(np.array(angular_errors) ** 2))

        # Generate Board Model
        # Anchor is at (0,0,0) with no rotation
        d = self.tag_size / 2.0
        corners_anchor = [[-d, d, 0.0], [d, d, 0.0], [d, -d, 0.0], [-d, -d, 0.0]]

        # Target is transformed by avg_translation and avg_rotation
        base_corners = np.array(
            [[-d, d, 0], [d, d, 0], [d, -d, 0], [-d, -d, 0]], dtype=np.float32
        )
        corners_target = []
        for point in base_corners:
            transformed = avg_rot_matrix @ point + avg_translation
            corners_target.append(transformed.tolist())

        # Print Metrics
        self.get_logger().info("\n" + "=" * 50)
        self.get_logger().info(" VALIDATION METRICS")
        self.get_logger().info("=" * 50)
        self.get_logger().info(f"Frames Processed:   {len(self.captured_frames)}")
        self.get_logger().info(f"Translation (XYZ):  {avg_translation} meters")
        self.get_logger().info(f"Translation StdDev: {std_translation * 1000} mm")
        self.get_logger().info(
            f"Rotation Jitter:    {rmse_rotation:.4f} degrees (RMSE)"
        )

        if np.any(std_translation > 0.01):  # Warning if > 1cm jitter
            self.get_logger().warn(
                "\nWARNING: High translation variance! Check camera calibration or hold steady."
            )
        else:
            self.get_logger().info("\nSUCCESS: Calibration looks stable.")

        # Save to YAML
        output_data = {
            "board_description": "Custom Angled Aruco Board",
            "ids": [int(self.id_anchor), int(self.id_target)],
            "tag_size": float(self.tag_size),
            "metrics": {
                "translation_std_mm": (std_translation * 1000).tolist(),
                "rotation_rmse_deg": float(rmse_rotation),
            },
            "corners": [corners_anchor, corners_target],
        }

        with open(self.output_board_yaml, "w") as outfile:
            yaml.dump(output_data, outfile, default_flow_style=None)

        self.get_logger().info(f"\nModel saved to {self.output_board_yaml}")

        # Shutdown the node
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = GenerateArucoBoardConfigZed()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
