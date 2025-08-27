import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from interfaces.msg import ArucoMarkers
from geometry_msgs.msg import Point, PoseArray, Pose

import cv2
import numpy as np


class ZEDArucoDetector(Node):
    def __init__(self):
        super().__init__("ZEDArucoDetector")

        self.cv_bridge = CvBridge()
        self.colour_image = None
        self.depth_image = None
        self.img_processed_flag = True
        self.header_msg = None
        self.fx = 0  # Focal length in x-direction
        self.fy = 0  # Focal length in y-direction
        self.cx = 0  # Principal point x-coordinate
        self.cy = 0  # Principal point y-coordinate
        self._aruco_detector_params = cv2.aruco.DetectorParameters_create()
        self._aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_250)

        # Subscribe to the ZED colour depth image topic
        self.create_subscription(
            Image, "/zed/zed_node/rgb/image_rect_color", self.rbg_image_callback, 10
        )

        self.create_subscription(
            Image, "/zed/zed_node/depth/depth_registered", self.depth_image_callback, 10
        )

        self.create_subscription(
            CameraInfo, "/zed/zed_node/rgb/camera_info", self.camera_info_callback, 10
        )

        self.create_timer(0.4, self.process_image)

        self.aruco_pub = self.create_publisher(
            ArucoMarkers, "/computer_vision/aruco_markers", 10
        )
        self.aruco_pose_publisher = self.create_publisher(
            PoseArray, "/computer_vision/aruco_marker_poses_viz", 10
        )

        self.get_logger().info("ZED ArUco Detector Node Initialized")

    def rbg_image_callback(self, msg: Image):
        # Use ros opencv bridge to convert the ROS image message to OpenCV format
        self.colour_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgra8")
        self.header_msg = msg.header
        self.img_processed_flag = False

    def depth_image_callback(self, msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )

    def camera_info_callback(self, msg: CameraInfo):
        # Gets camera intrinsic parameters
        self.fx = msg.k[0]  # Focal length in x-direction
        self.fy = msg.k[4]  # Focal length in y-direction
        self.cx = msg.k[2]  # Principal point x-coordinate
        self.cy = msg.k[5]  # Principal point y-coordinate

    def process_image(self):
        if self.img_processed_flag:
            return
        if self.colour_image is None or self.depth_image is None:
            self.get_logger().warn("Colour or depth image not received yet")
            return

        self.img_processed_flag = True

        # Remove the last channel of the image to get bgr image
        colour_image = self.colour_image[:, :, :3]

        # Detect ArUco markers in the image
        corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
            colour_image, self._aruco_dict, parameters=self._aruco_detector_params
        )

        # Create an ArucoMarkers message
        aruco_markers_msg = ArucoMarkers()
        aruco_markers_msg.header = self.header_msg

        # Loop through detected markers
        if ids is not None:
            self.get_logger().info(f"Detected {len(ids)} markers")
            for i in range(len(corners)):
                single_tag_corners = corners[i][0]
                single_tag_id = ids[i][0]

                # Calculate the 3D coordinates of the marker
                x, y, z = self.calculate_point(single_tag_corners, self.depth_image)

                if z is None:
                    continue

                # Append marker ID and 3D coordinates to the message
                self.get_logger().info(
                    f"Marker ID: {single_tag_id}, Coordinates: ({x}, {y}, {z}). ztype: {type(z)}"
                )
                aruco_markers_msg.marker_ids.append(single_tag_id)
                aruco_markers_msg.points.append(Point(x=x, y=y, z=z))
                aruco_markers_msg.is_moving.append(False)

        # Publish the ArucoMarkers message
        self.aruco_pub.publish(aruco_markers_msg)

        # Create a PoseArray message for visualization
        pose_array_msg = PoseArray()
        pose_array_msg.header = self.header_msg
        for i in range(len(aruco_markers_msg.marker_ids)):
            pose = Pose()
            pose.position = aruco_markers_msg.points[i]
            pose.orientation.w = 1.0
            pose_array_msg.poses.append(pose)

        self.aruco_pose_publisher.publish(pose_array_msg)

    def calculate_point(self, single_tag_corners, dep_img):
        # Get the bounding box of the detected marker
        x_min = int(min(single_tag_corners[:, 0]))
        x_max = int(max(single_tag_corners[:, 0]))
        y_min = int(min(single_tag_corners[:, 1]))
        y_max = int(max(single_tag_corners[:, 1]))

        # Get the depth values within the bounding box
        depth_values = dep_img[y_min:y_max, x_min:x_max].astype(np.float32)

        # Remove invalid depth values
        valid_depth_values = depth_values[np.isfinite(depth_values)]

        if valid_depth_values.size == 0:
            self.get_logger().warn("No valid depth values found in the bounding box")
            return 0, 0, None

        self.get_logger().info(f"Depth values: {valid_depth_values}")

        # Calculate the average depth value
        avg_depth = np.mean(valid_depth_values)

        if avg_depth == 0 or not np.isfinite(avg_depth):
            self.get_logger().warn(
                "Average depth is zero or not finite, skipping marker"
            )
            return 0, 0, None

        center_x = int((x_min + x_max) / 2)
        center_y = int((y_min + y_max) / 2)

        # Calculate the 3D coordinates of the marker
        x = (center_x - self.cx) * avg_depth / self.fx
        y = (center_y - self.cy) * avg_depth / self.fy
        z = avg_depth

        return float(x), float(y), float(z)


def main(args=None):
    rclpy.init(args=args)
    node = ZEDArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
