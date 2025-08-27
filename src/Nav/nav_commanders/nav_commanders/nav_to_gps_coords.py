import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time
from robot_localization.srv import FromLL
from rclpy.node import Node
from .gps_utils import latLonYaw2Geopose
from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPose
from interfaces.srv import NavToGPSGeopose
from time import sleep


class GpsCommander(Node):
    """
    Class to use nav2 gps waypoint follower to follow a set of waypoints logged in a yaml file
    """

    def __init__(self):
        super().__init__("GpsCommander")
        self.navigator = BasicNavigator("GpsCommander_Navigator")

        self.nav_fix_topic = "fromLL"
        self.geopose_service_name = "commander/nav_to_gps_geopose"

        self.localizer_callback_group = MutuallyExclusiveCallbackGroup()
        self.localizer = self.create_client(
            FromLL, "fromLL", callback_group=self.localizer_callback_group
        )
        self.geopose_service = self.create_service(
            NavToGPSGeopose, self.geopose_service_name, self.geopose_server
        )

        self.get_logger().info("Nav fix servie name: " + str("fromLL"))

        self.get_logger().info("Waiting for nav_sat to be active")
        count = 0
        while not self.localizer.wait_for_service(timeout_sec=2.0):
            count += 1
            if count % 10 == 1:
                self.get_logger().info(
                    f"Service on {self.nav_fix_topic} is not available, waiting again..."
                )
        self.get_logger().info("nav_sat is active")

        self.get_logger().info("Waiting for Nav2 to be active")
        self.navigator.waitUntilNav2Active(localizer="controller_server")
        self.get_logger().info("Nav2 is active")

    def geopose_server(
        self, msg: NavToGPSGeopose, response: NavToGPSGeopose
    ) -> NavToGPSGeopose:
        self.get_logger().info("Recieved a new gps goal")

        self.req = FromLL.Request()
        self.req.ll_point.longitude = msg.goal.position.longitude
        self.req.ll_point.latitude = msg.goal.position.latitude
        self.req.ll_point.altitude = msg.goal.position.altitude

        self.get_logger().info(
            f"Long={self.req.ll_point.longitude:.8f}, Lat={self.req.ll_point.latitude:.8f}, Alt={self.req.ll_point.altitude:.8f}"
        )
        self.get_logger().info(f"Request is: {repr(self.req)}")

        self.get_logger().info(f"Calling nav_sat service to transform geopose to map")
        self.future = self.localizer.call_async(self.req)
        # self.localizer_callback_group.spin_until_future_complete(self, self.future)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info(f"Got pose in map frame")

        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "map"
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.target_pose.pose.position = self.future.result().map_point

        self.get_logger().info(
            f"Target Pose in the map frame: x={self.future.result().map_point.x:.8f}, y={self.future.result().map_point.y:.8f}, z={self.future.result().map_point.z:.8f}"
        )

        self.target_pose.pose.orientation = msg.goal.orientation

        response.success = self.navigator.goToPose(self.target_pose)

        return response


def main(args=None):
    rclpy.init(args=args)

    node = GpsCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
