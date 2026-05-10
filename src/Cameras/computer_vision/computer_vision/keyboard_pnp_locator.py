#!/usr/bin/env python3
"""Keyboard locator using per-marker PnP pose estimation.

Detects ArUco markers affixed to the keyboard corners and calls
cv2.aruco.estimatePoseSingleMarkers on each visible marker, then averages
the resulting poses to produce a stable keyboard pose in the camera frame.

This approach requires only the marker size (2 cm per URC 2026 rules) and
does NOT depend on knowing the exact inter-marker distances in advance.

Camera calibration is loaded from a YAML file at startup so the node works
with the wireless USB arm camera, which does not publish a CameraInfo topic.
The frame is pre-undistorted before ArUco detection to improve corner
detection quality under high radial distortion (k1 ≈ 0.60).

The resulting pose is broadcast as a TF transform:
    parent : camera optical frame (from image header)
    child  : keyboard_center_link  (configurable via 'output_frame' param)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation

import cv2
import numpy as np
import yaml

from . import cv2_utils


class KeyboardPnPLocator(Node):
    """Estimates keyboard pose from ArUco markers via per-marker PnP."""

    MIN_MARKERS = 2

    def __init__(self):
        super().__init__("keyboard_pnp_locator")

        self.declare_parameter("config_file", "config/keyboard_board_config.yaml")
        self.declare_parameter(
            "camera_calibration_file", "config/end_effector_calibration.yaml"
        )
        self.declare_parameter("output_frame", "keyboard_center_link")
        self.declare_parameter("image_topic", "/EndEffector/image_raw")
        self.declare_parameter("camera_info_topic", "")  # empty = don't subscribe
        # URC 2026 navigation posts use DICT_4X4_50; keyboard markers likely the same.
        # Override via launch argument if the organizers specify otherwise.
        self.declare_parameter("aruco_dict", "DICT_4X4_50")

        config_file = self.get_parameter("config_file").value
        calib_file = self.get_parameter("camera_calibration_file").value
        self.output_frame = self.get_parameter("output_frame").value
        image_topic = self.get_parameter("image_topic").value
        camera_info_topic = self.get_parameter("camera_info_topic").value
        aruco_dict_name = self.get_parameter("aruco_dict").value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.bridge = CvBridge()

        # Undistortion maps — pre-computed once from calibration file.
        # After remapping, we use new_camera_matrix with zero distortion.
        self._map_x: np.ndarray | None = None
        self._map_y: np.ndarray | None = None
        self._pose_camera_matrix: np.ndarray | None = (
            None  # new K for undistorted frame
        )

        self._aruco_dict = cv2_utils.get_aruco_dictionary(aruco_dict_name)
        self._aruco_params = cv2_utils.create_detector_parameters()
        self._marker_size, self._valid_ids = self._load_board_config(config_file)
        self._load_calibration(calib_file)

        # Optional CameraInfo subscription — only wired up when topic is set.
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

        id_info = self._valid_ids.tolist() if self._valid_ids is not None else "any"
        self.get_logger().info(
            f"Keyboard PnP locator ready — dict={aruco_dict_name}, "
            f"marker_size={self._marker_size * 100:.1f} cm, "
            f"valid IDs={id_info} — waiting for images..."
        )

    # ------------------------------------------------------------------
    # Config / Calibration loading
    # ------------------------------------------------------------------

    def _load_board_config(self, config_file: str) -> tuple[float, np.ndarray | None]:
        with open(config_file, "r") as f:
            cfg = yaml.safe_load(f)
        marker_size = float(cfg["marker_size"])
        # `ids` is optional. If omitted, all detected markers are accepted —
        # useful when the competition has not yet published the marker IDs.
        valid_ids = np.array(cfg["ids"], dtype=np.int32) if "ids" in cfg else None
        return marker_size, valid_ids

    def _load_calibration(self, calib_file: str) -> None:
        """Load K and D from a YAML calibration file and pre-compute undistortion maps."""
        with open(calib_file, "r") as f:
            cfg = yaml.safe_load(f)

        w = int(cfg["image_width"])
        h = int(cfg["image_height"])

        K = np.array(cfg["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
        D = np.array(cfg["distortion_coefficients"]["data"], dtype=np.float64)

        # alpha=1 keeps all original pixels so corner markers near edges aren't clipped.
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
            f"Loaded calibration from {calib_file} — "
            f"image {w}x{h}, D coeffs={len(D)}, undistortion maps ready"
        )

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Optional: recompute undistortion maps if a CameraInfo topic is available."""
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

    def _image_callback(self, msg: Image):
        if self._map_x is None:
            self.get_logger().warn(
                "Calibration not loaded yet, skipping frame",
                throttle_duration_sec=2.0,
            )
            return

        raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Pre-undistort the frame — improves ArUco corner detection under
        # high radial distortion (k1 ≈ 0.60 for this camera).
        undistorted = cv2.remap(raw, self._map_x, self._map_y, cv2.INTER_LINEAR)

        corners, ids, _ = cv2_utils.detect_markers(
            undistorted, self._aruco_dict, self._aruco_params
        )

        if ids is None or len(ids) < self.MIN_MARKERS:
            self.get_logger().warn(
                f"Only {0 if ids is None else len(ids)} marker(s) visible "
                f"(need {self.MIN_MARKERS}) — skipping",
                throttle_duration_sec=1.0,
            )
            return

        # Filter to known IDs when configured; otherwise accept everything.
        if self._valid_ids is not None:
            mask = np.isin(ids.flatten(), self._valid_ids)
            if mask.sum() < self.MIN_MARKERS:
                self.get_logger().warn(
                    f"Only {mask.sum()} known marker(s) visible — skipping",
                    throttle_duration_sec=1.0,
                )
                return
            corners = [c for c, m in zip(corners, mask) if m]
            ids = ids[mask]

        # Frame is already undistorted → pass new_K and zero distortion.
        zero_D = np.zeros(5, dtype=np.float64)
        rvecs, tvecs = cv2_utils.estimate_pose_single_markers(
            corners, self._marker_size, self._pose_camera_matrix, zero_D
        )

        rvec_mean, tvec_mean = cv2_utils.average_poses(rvecs, tvecs)

        self._broadcast_tf(msg.header, rvec_mean, tvec_mean)
        t = tvec_mean.flatten()
        self.get_logger().info(
            f"Keyboard pose [{len(ids)} markers] — "
            f"pos=({t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}) m",
            throttle_duration_sec=0.5,
        )

    # ------------------------------------------------------------------
    # TF broadcast
    # ------------------------------------------------------------------

    def _broadcast_tf(self, header, rvec: np.ndarray, tvec: np.ndarray):
        rot_mat, _ = cv2.Rodrigues(rvec)
        quat = Rotation.from_matrix(rot_mat).as_quat()  # [x, y, z, w]
        t = tvec.flatten()

        tf_msg = TransformStamped()
        tf_msg.header = header
        tf_msg.child_frame_id = self.output_frame

        tf_msg.transform.translation.x = float(t[0])
        tf_msg.transform.translation.y = float(t[1])
        tf_msg.transform.translation.z = float(t[2])
        tf_msg.transform.rotation.x = float(quat[0])
        tf_msg.transform.rotation.y = float(quat[1])
        tf_msg.transform.rotation.z = float(quat[2])
        tf_msg.transform.rotation.w = float(quat[3])

        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardPnPLocator()
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
