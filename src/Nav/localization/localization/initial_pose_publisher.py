#!/usr/bin/env python3
"""
Initial Pose Publisher for breaking EKF circular dependency.

This node publishes an initial origin pose at startup to allow the EKF to
begin publishing transforms. Once SLAM Toolbox starts providing poses, 
it will naturally take over as the more accurate pose source.

The pose is published only once (or a few times) at startup to bootstrap the system.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy


class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__("initial_pose_publisher")

        # Parameters
        self.declare_parameter("frame_id", "map")
        self.declare_parameter(
            "publish_count", 5
        )  # Publish a few times to ensure receipt
        self.declare_parameter("publish_rate", 1.0)  # Hz
        self.declare_parameter("x", 0.0)
        self.declare_parameter("y", 0.0)
        self.declare_parameter("z", 0.0)
        self.declare_parameter("yaw", 0.0)  # radians

        self.frame_id = self.get_parameter("frame_id").value
        self.publish_count = self.get_parameter("publish_count").value
        self.publish_rate = self.get_parameter("publish_rate").value
        self.x = self.get_parameter("x").value
        self.y = self.get_parameter("y").value
        self.z = self.get_parameter("z").value
        self.yaw = self.get_parameter("yaw").value

        # Publisher with latching-like behavior using transient local durability
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/ekf_initial_pose", qos
        )

        self.count = 0
        self.timer = self.create_timer(
            1.0 / self.publish_rate, self.publish_initial_pose
        )

        self.get_logger().info(
            f"Initial Pose Publisher started. Will publish origin pose {self.publish_count} times."
        )

    def publish_initial_pose(self):
        """Publish an initial pose at the origin to bootstrap the EKF."""
        if self.count >= self.publish_count:
            self.get_logger().info(
                "Finished publishing initial poses. Node will continue running."
            )
            self.timer.cancel()
            return

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        # Set position
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.position.z = self.z

        # Set orientation (yaw only, roll and pitch = 0)
        # Convert yaw to quaternion: qz = sin(yaw/2), qw = cos(yaw/2)
        import math

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(self.yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self.yaw / 2.0)

        # Set a high covariance to indicate low confidence
        # This ensures SLAM will quickly override it with better estimates
        # Format: [x, y, z, rot_x, rot_y, rot_z]
        covariance = [0.0] * 36
        covariance[0] = 10.0  # x variance - high uncertainty
        covariance[7] = 10.0  # y variance - high uncertainty
        covariance[14] = 10.0  # z variance - high uncertainty
        covariance[21] = 1.0  # roll variance
        covariance[28] = 1.0  # pitch variance
        covariance[35] = 1.0  # yaw variance - moderate uncertainty
        msg.pose.covariance = covariance

        self.pose_pub.publish(msg)
        self.count += 1

        self.get_logger().info(
            f"Published initial pose ({self.count}/{self.publish_count}): "
            f"x={self.x:.2f}, y={self.y:.2f}, yaw={self.yaw:.2f} rad"
        )


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
