#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path


class SearchPatternVisualizer(Node):
    """
    A simple ROS2 node that generates a lawnmower search pattern
    and continuously publishes it as a Path message for RViz visualization.
    """

    def __init__(self):
        super().__init__("search_pattern_visualizer")

        # Publisher for the path topic
        self.path_pub = self.create_publisher(Path, "/search_pattern_path", 10)

        # Grid parameters (Modify these to instantly see changes in RViz)
        self.search_size = 20.0  # 20x20 meter grid
        self.lane_spacing = 4.0  # 4 meters between lanes

        # Publish at 1Hz so it remains persistent in RViz
        self.timer = self.create_timer(1.0, self.publish_path)
        self.get_logger().info(
            "Publishing search pattern to '/search_pattern_path' at 1Hz..."
        )

    def yaw_to_quaternion(self, yaw):
        """Converts a yaw angle (in radians) to a geometry_msgs Quaternion."""
        q = Quaternion()
        q.x, q.y = 0.0, 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def publish_path(self):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()

        # Center of the search area (0,0 for visualization purposes)
        center_x, center_y = 0.0, 0.0
        half_size = self.search_size / 2.0

        start_x = center_x - half_size
        start_y = center_y - half_size
        num_lanes = int(self.search_size / self.lane_spacing) + 1

        for i in range(num_lanes):
            current_y = start_y + (i * self.lane_spacing)

            # Alternate directions for the boustrophedon sweep
            if i % 2 == 0:
                x_start, x_end, yaw = start_x, start_x + self.search_size, 0.0
            else:
                x_start, x_end, yaw = start_x + self.search_size, start_x, math.pi

            # Entry point of the current lane
            pose_start = PoseStamped()
            pose_start.header = path_msg.header
            pose_start.pose.position.x = x_start
            pose_start.pose.position.y = current_y
            pose_start.pose.orientation = self.yaw_to_quaternion(yaw)
            path_msg.poses.append(pose_start)

            # Exit point of the current lane
            pose_end = PoseStamped()
            pose_end.header = path_msg.header
            pose_end.pose.position.x = x_end
            pose_end.pose.position.y = current_y
            pose_end.pose.orientation = self.yaw_to_quaternion(yaw)
            path_msg.poses.append(pose_end)

        # Send to RViz
        self.path_pub.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SearchPatternVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
