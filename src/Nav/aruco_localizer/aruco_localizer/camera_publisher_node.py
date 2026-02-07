#!/usr/bin/env python3
"""
Camera Publisher Node

This node opens a camera using OpenCV with V4L2 backend, reads camera calibration
from a YAML file, and publishes both the camera images and camera_info to ROS2 topics.

Published Topics:
    /camera/image_raw (sensor_msgs.msg.Image)
       Raw camera images
    
    /camera/camera_info (sensor_msgs.msg.CameraInfo)
       Camera calibration information

Parameters:
    camera_device - camera device path or index (e.g., "/dev/video0" or "0")
    camera_name - name of the camera
    image_width - image width in pixels
    image_height - image height in pixels
    frame_rate - publishing frame rate in Hz
    image_topic - topic to publish images
    camera_info_topic - topic to publish camera info
    camera_frame_id - frame ID for the camera
    calibration_file - path to camera calibration YAML file
    use_compressed - whether to also publish compressed images

Author: GitHub Copilot
Version: 2025-12-05
"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

import cv2
import numpy as np
import yaml
import os

# ROS2 message imports
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge


class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__("camera_publisher_node")

        self.initialize_parameters()

        # Initialize cv_bridge
        self.bridge = CvBridge()

        # Load camera calibration
        self.camera_info_msg = None
        if self.calibration_file:
            self.load_camera_calibration()

        # Open camera with V4L2 backend
        self.cap = None
        self.open_camera()

        # Set up publishers
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, self.camera_info_topic, 10
        )

        if self.use_compressed:
            self.compressed_pub = self.create_publisher(
                CompressedImage, f"{self.image_topic}/compressed", 10
            )

        # Create timer for publishing at specified frame rate
        timer_period = 1.0 / self.frame_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f"Camera Publisher Node initialized: {self.camera_device} @ {self.frame_rate} Hz"
        )
        self.get_logger().info(
            f"Publishing to: {self.image_topic}, {self.camera_info_topic}"
        )

    def initialize_parameters(self):
        """Declare and read ROS2 parameters"""

        self.declare_parameter(
            name="camera_device",
            value="/dev/video0",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Camera device path (e.g., /dev/video0) or index (e.g., 0)",
            ),
        )

        self.declare_parameter(
            name="camera_name",
            value="camera",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING, description="Name of the camera"
            ),
        )

        self.declare_parameter(
            name="image_width",
            value=1280,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Image width in pixels",
            ),
        )

        self.declare_parameter(
            name="image_height",
            value=720,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_INTEGER,
                description="Image height in pixels",
            ),
        )

        self.declare_parameter(
            name="frame_rate",
            value=30.0,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_DOUBLE,
                description="Publishing frame rate in Hz",
            ),
        )

        self.declare_parameter(
            name="image_topic",
            value="/camera/image_raw",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish raw images",
            ),
        )

        self.declare_parameter(
            name="camera_info_topic",
            value="/camera/camera_info",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Topic to publish camera info",
            ),
        )

        self.declare_parameter(
            name="camera_frame_id",
            value="camera_optical_frame",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Frame ID for the camera",
            ),
        )

        self.declare_parameter(
            name="calibration_file",
            value="",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="Path to camera calibration YAML file",
            ),
        )

        self.declare_parameter(
            name="use_compressed",
            value=False,
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_BOOL,
                description="Whether to also publish compressed images",
            ),
        )

        self.declare_parameter(
            name="fourcc",
            value="MJPG",
            descriptor=ParameterDescriptor(
                type=ParameterType.PARAMETER_STRING,
                description="FourCC code for video format (e.g., MJPG, YUYV)",
            ),
        )

        # Get parameter values
        self.camera_device = self.get_parameter("camera_device").value
        self.camera_name = self.get_parameter("camera_name").value
        self.image_width = self.get_parameter("image_width").value
        self.image_height = self.get_parameter("image_height").value
        self.frame_rate = self.get_parameter("frame_rate").value
        self.image_topic = self.get_parameter("image_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.camera_frame_id = self.get_parameter("camera_frame_id").value
        self.calibration_file = self.get_parameter("calibration_file").value
        self.use_compressed = self.get_parameter("use_compressed").value
        self.fourcc = self.get_parameter("fourcc").value

    def open_camera(self):
        """Open camera with OpenCV using V4L2 backend"""
        try:
            # Parse camera device (could be string path or integer index)
            try:
                device = int(self.camera_device)
            except ValueError:
                device = self.camera_device

            # Open camera with V4L2 backend
            self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)

            if not self.cap.isOpened():
                self.get_logger().error(f"Failed to open camera: {self.camera_device}")
                raise RuntimeError(f"Cannot open camera {self.camera_device}")

            # Set camera properties
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)

            # Set FourCC if specified
            if self.fourcc and len(self.fourcc) == 4:
                fourcc_code = cv2.VideoWriter_fourcc(*self.fourcc)
                self.cap.set(cv2.CAP_PROP_FOURCC, fourcc_code)

            # Read actual camera properties
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = self.cap.get(cv2.CAP_PROP_FPS)

            self.get_logger().info(f"Camera opened: {self.camera_device}")
            self.get_logger().info(f"Resolution: {actual_width}x{actual_height}")
            self.get_logger().info(f"FPS: {actual_fps}")

            # Warm up camera by reading a few frames
            for _ in range(5):
                self.cap.read()

        except Exception as e:
            self.get_logger().error(f"Error opening camera: {e}")
            raise

    def load_camera_calibration(self):
        """Load camera calibration from YAML file"""
        try:
            if not os.path.exists(self.calibration_file):
                self.get_logger().warn(
                    f"Calibration file not found: {self.calibration_file}. "
                    "Publishing images without camera_info."
                )
                return

            with open(self.calibration_file, "r") as f:
                calib_data = yaml.safe_load(f)

            # Create CameraInfo message
            self.camera_info_msg = CameraInfo()

            # Set basic info
            self.camera_info_msg.width = calib_data.get("image_width", self.image_width)
            self.camera_info_msg.height = calib_data.get(
                "image_height", self.image_height
            )

            # Set distortion model
            self.camera_info_msg.distortion_model = calib_data.get(
                "distortion_model", "plumb_bob"
            )

            # Set camera matrix (K)
            if "camera_matrix" in calib_data:
                k_data = calib_data["camera_matrix"]["data"]
                self.camera_info_msg.k = k_data

            # Set distortion coefficients (D)
            if "distortion_coefficients" in calib_data:
                d_data = calib_data["distortion_coefficients"]["data"]
                self.camera_info_msg.d = d_data

            # Set rectification matrix (R)
            if "rectification_matrix" in calib_data:
                r_data = calib_data["rectification_matrix"]["data"]
                self.camera_info_msg.r = r_data
            else:
                # Identity matrix if not specified
                self.camera_info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]

            # Set projection matrix (P)
            if "projection_matrix" in calib_data:
                p_data = calib_data["projection_matrix"]["data"]
                self.camera_info_msg.p = p_data

            self.get_logger().info(
                f"Loaded camera calibration from: {self.calibration_file}"
            )
            self.get_logger().info(
                f"Camera: {calib_data.get('camera_name', 'unknown')}"
            )

        except Exception as e:
            self.get_logger().error(f"Error loading calibration file: {e}")
            self.camera_info_msg = None

    def timer_callback(self):
        """Read frame from camera and publish"""
        try:
            ret, frame = self.cap.read()

            if not ret:
                self.get_logger().warn(
                    "Failed to read frame from camera", throttle_duration_sec=1.0
                )
                return

            # Create timestamp
            timestamp = self.get_clock().now().to_msg()

            # Convert frame to RGB (OpenCV uses BGR by default)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Create and publish Image message
            img_msg = self.bridge.cv2_to_imgmsg(frame_rgb, encoding="rgb8")
            img_msg.header.stamp = timestamp
            img_msg.header.frame_id = self.camera_frame_id
            self.image_pub.publish(img_msg)

            # Publish compressed image if enabled
            if self.use_compressed:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = timestamp
                compressed_msg.header.frame_id = self.camera_frame_id
                compressed_msg.format = "jpeg"
                compressed_msg.data = np.array(cv2.imencode(".jpg", frame)[1]).tobytes()
                self.compressed_pub.publish(compressed_msg)

            # Publish camera info if available
            if self.camera_info_msg is not None:
                self.camera_info_msg.header.stamp = timestamp
                self.camera_info_msg.header.frame_id = self.camera_frame_id
                self.camera_info_pub.publish(self.camera_info_msg)

        except Exception as e:
            self.get_logger().error(
                f"Error in timer callback: {e}", throttle_duration_sec=1.0
            )

    def destroy_node(self):
        """Clean up resources"""
        if self.cap is not None and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("Camera released")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(
            "Shutting down camera_publisher_node due to KeyboardInterrupt"
        )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
