#!/usr/bin/env python3
"""Navigation marker locator using per-marker PnP pose estimation.

Detects ArUco navigation markers in an image stream, estimates a pose for

each marker via cv2.aruco.estimatePoseSingleMarkers (or fallback), and
publishes the marker centers as an ArucoMarkers message plus a PoseArray
for visualization.

This node mirrors the keyboard PnP locator design for calibration loading
and image handling, while using the ZED ArUco detector's publication style.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Point, PoseArray, Pose
from interfaces.msg import ArucoMarkers
from cv_bridge import CvBridge

import cv2
import numpy as np
import yaml

from . import cv2_utils


class NavMarkerLocator(Node):
    """Publishes detected navigation markers as 3D points in camera frame."""

    def __init__(self):
        super().__init__("nav_marker_locator")

        self.declare_parameter("config_file", "config/nav_marker_config.yaml")
        self.declare_parameter(
            "camera_calibration_file", "config/drive_calibration.yaml"
        )
        self.declare_parameter("image_topic", "/Drive/image_raw")
        self.declare_parameter("camera_info_topic", "")  # empty = don't subscribe
        self.declare_parameter("aruco_dict", "DICT_4X4_50")

        config_file = self.get_parameter("config_file").value
        calib_file = self.get_parameter("camera_calibration_file").value
        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        aruco_dict_name = self.get_parameter("aruco_dict").value

        self.bridge = CvBridge()
        self._aruco_dict = cv2_utils.get_aruco_dictionary(aruco_dict_name)
        self._aruco_params = cv2_utils.create_detector_parameters()

        self._map_x: np.ndarray | None = None
        self._map_y: np.ndarray | None = None
        self._pose_camera_matrix: np.ndarray | None = None
        self._marker_size, self._valid_ids = self._load_board_config(config_file)
        self._load_calibration(calib_file)

        if camera_info_topic:
            self.create_subscription(
                CameraInfo,
                camera_info_topic,
                self._camera_info_callback,
                qos_profile_sensor_data,
            )

        self.create_subscription(
            Image,
            image_topic,
            self._image_callback,
            qos_profile_sensor_data,
        )

        self.marker_pub = self.create_publisher(
            ArucoMarkers, "/computer_vision/nav_markers", 10
        )
        self.marker_pose_pub = self.create_publisher(
            PoseArray, "/computer_vision/nav_marker_poses_viz", 10
        )

        id_info = self._valid_ids.tolist() if self._valid_ids is not None else "any"
        self.get_logger().info(
            f"Nav marker locator ready — dict={aruco_dict_name}, "
            f"marker_size={self._marker_size * 100:.1f} cm, valid IDs={id_info}"
        )

    # ------------------------------------------------------------------
    # Config / Calibration loading
    # ------------------------------------------------------------------

    def _load_board_config(self, config_file: str) -> tuple[float, np.ndarray | None]:
        with open(config_file, "r") as f:
            cfg = yaml.safe_load(f)
        marker_size = float(cfg["marker_size"])
        valid_ids = np.array(cfg["ids"], dtype=np.int32) if "ids" in cfg else None
        return marker_size, valid_ids

    def _load_calibration(self, calib_file: str) -> None:
        with open(calib_file, "r") as f:
            cfg = yaml.safe_load(f)

        w = int(cfg["image_width"])
        h = int(cfg["image_height"])

        K = np.array(cfg["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        D = np.array(cfg["distortion_coefficients"]["data"], dtype=np.float64)

        new_K, _ = cv2.getOptimalNewCameraMatrix(
            K, D, (w, h), alpha=1, newImgSize=(w, h)
        )
        map_x, map_y = cv2.initUndistortRectifyMap(
            K, D, None, new_K, (w, h), cv2.CV_32FC1
        )

        self._map_x = map_x
        self._map_y = map_y
        self._pose_camera_matrix = new_K

        self.get_logger().info(
            f"Loaded calibration from {calib_file} — image {w}x{h}, "
            f"D coeffs={len(D)}, undistortion maps ready"
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        w, h = msg.width, msg.height

        new_K, _ = cv2.getOptimalNewCameraMatrix(
            K, D, (w, h), alpha=1, newImgSize=(w, h)
        )
        self._map_x, self._map_y = cv2.initUndistortRectifyMap(
            K, D, None, new_K, (w, h), cv2.CV_32FC1
        )
        self._pose_camera_matrix = new_K

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _image_callback(self, msg: Image) -> None:
        if self._map_x is None:
            self.get_logger().warn(
                "Calibration not loaded yet, skipping frame",
                throttle_duration_sec=2.0,
            )
            return

        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        undistorted = cv2.remap(raw, self._map_x, self._map_y, cv2.INTER_LINEAR)

        corners, ids, _ = cv2_utils.detect_markers(
            undistorted, self._aruco_dict, self._aruco_params
        )

        marker_msg = ArucoMarkers()
        marker_msg.header = msg.header

        pose_array = PoseArray()
        pose_array.header = msg.header

        if ids is None or len(ids) == 0:
            self.marker_pub.publish(marker_msg)
            self.marker_pose_pub.publish(pose_array)
            return

        if self._valid_ids is not None:
            mask = np.isin(ids.flatten(), self._valid_ids)
            if not mask.any():
                self.marker_pub.publish(marker_msg)
                self.marker_pose_pub.publish(pose_array)
                return
            corners = [c for c, m in zip(corners, mask) if m]
            ids = ids[mask]

        zero_D = np.zeros(5, dtype=np.float64)
        rvecs, tvecs = cv2_utils.estimate_pose_single_markers(
            corners, self._marker_size, self._pose_camera_matrix, zero_D
        )

        for marker_id, tvec in zip(ids.flatten(), tvecs):
            t = tvec.flatten()
            marker_msg.marker_ids.append(int(marker_id))
            marker_msg.points.append(Point(x=float(t[0]), y=float(t[1]), z=float(t[2])))
            marker_msg.is_moving.append(False)

            pose = Pose()
            pose.position.x = float(t[0])
            pose.position.y = float(t[1])
            pose.position.z = float(t[2])
            pose.orientation.w = 1.0
            pose_array.poses.append(pose)

        self.marker_pub.publish(marker_msg)
        self.marker_pose_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = NavMarkerLocator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
