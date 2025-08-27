import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time
from robot_localization.srv import FromLL
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from geographic_msgs.msg import GeoPose
from interfaces.srv import NavToGPSGeopose
from std_msgs.msg import Int8
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
import math
from enum import Enum, auto
from threading import Event


class MissionState(Enum):
    DO_NOTHING = auto()
    NAV_TO_INTERMEDIATE_GOAL = auto()
    NAV_TO_FINAL_GOAL = auto()
    NAV_TO_ARUCO_MARKER = auto()


class IncrementalGpsCommander(Node):
    """
    Class to incrementally navigate to a GPS goal.
    """

    def __init__(self):
        super().__init__("incremental_gps_commander")
        self.navigator = BasicNavigator("incremental_gps_navigator")
        self.mission_state = MissionState.DO_NOTHING

        self.declare_parameter("look_for_aruco", False)
        self.lookForAruco = (
            self.get_parameter("look_for_aruco").get_parameter_value().bool_value
        )
        self.get_logger().info(f"Look for aruco: {self.lookForAruco}")

        # Distance in meters to the intermediate goal
        self.declare_parameter("incremental_distance", 25.0)
        self.incremental_distance = (
            self.get_parameter("incremental_distance")
            .get_parameter_value()
            .double_value
        )
        self.get_logger().info(
            f"Incremental distance set to: {self.incremental_distance} meters"
        )

        self.declare_parameter("frequency", 1 / 30.0)  # Frequency in Hz
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.get_logger().info(
            f"Frequency set to: {self.frequency} Hz or {1.0 / self.frequency} seconds"
        )

        self.declare_parameter("robot_pose_topic", "odometry/filtered/global")
        self.robot_pose_topic = (
            self.get_parameter("robot_pose_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Robot pose topic set to: {self.robot_pose_topic}")

        self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        # self.i = 0
        self.nav_fix_topic = "fromLL"
        self.geopose_service_name = "commander/nav_to_gps_geopose"
        self.intermediate_goal_topic = "goal"
        self.lights_topic = "/light"
        self.nav_activate_light_code = 1
        self.nav_completed_light_code = 3
        # self.nav_fromll_done_event = Event()

        self.localizer_callback_group = MutuallyExclusiveCallbackGroup()
        self.pose_callback_group = ReentrantCallbackGroup()
        self.goal_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.lights_callback_group = MutuallyExclusiveCallbackGroup()

        self.localizer_client = self.create_client(
            FromLL, "fromLL", callback_group=self.localizer_callback_group
        )
        self.geopose_service = self.create_service(
            NavToGPSGeopose,
            self.geopose_service_name,
            self.geopose_server,
            callback_group=self.goal_callback_group,
        )
        self.robot_pose_subscription = self.create_subscription(
            Odometry,
            self.robot_pose_topic,
            self.robot_pose_callback,
            qos_profile_sensor_data,
            callback_group=self.pose_callback_group,
        )
        self.intermediate_goal_publisher = self.create_publisher(
            PoseStamped, self.intermediate_goal_topic, 5
        )
        self.lights_publisher = self.create_publisher(
            Int8,
            self.lights_topic,
            qos_profile=self.qos_profile,
            callback_group=self.lights_callback_group,
        )

        self.final_lat_lon = None
        self.current_robot_pose = None
        self.goal_handle = None  # To store the navigation goal handle
        self.intermediate_goal_handle = None

        self.timer = self.create_timer(
            1.0 / self.frequency, self.timer_callback, callback_group=self.timer_group
        )

        self.get_logger().info("Nav fix service name: " + str("fromLL"))
        self.get_logger().info("Waiting for nav_sat to be active")
        while not self.localizer_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                f"Service on {self.nav_fix_topic} is not available, waiting again..."
            )
        self.get_logger().info("nav_sat is active")

        self.get_logger().info("Waiting for Nav2 to be active")
        self.navigator.waitUntilNav2Active(localizer="controller_server")
        self.get_logger().info("Nav2 is active")

    def robot_pose_callback(self, msg: Odometry):
        """Callback to update the current robot pose."""
        self.current_robot_pose = msg.pose

        # target_pose = PoseStamped()
        # target_pose.header.frame_id = "map"
        # target_pose.header.stamp = self.get_clock().now().to_msg()
        # # target_pose.pose.position = result.map_point
        # # target_pose.pose.orientation = orientation
        # # self.get_logger().info(
        # #     f"Converted to map frame: x={target_pose.pose.position.x:.2f}, y={target_pose.pose.position.y:.2f}"
        # # )

        # # hack for indoor testing
        # target_pose.pose.position.x = 0.1 * self.i
        # target_pose.pose.position.y = 0.0
        # target_pose.pose.position.z = 0.0
        # self.i += 1

        # self.current_robot_pose = target_pose
        # self.get_logger().info(f"Current pose: {self.current_robot_pose.pose.position}")

    def geopose_server(
        self, msg: NavToGPSGeopose, response: NavToGPSGeopose
    ) -> NavToGPSGeopose:
        """Service server to receive the initial GPS goal."""
        self.get_logger().info(f"Received a new GPS goal: {msg}")
        self.final_lat_lon = (
            msg.goal.position.latitude,
            msg.goal.position.longitude,
            msg.goal.position.altitude,
            msg.goal.orientation,
        )
        self.reset()
        self.mission_state = MissionState.NAV_TO_INTERMEDIATE_GOAL
        response.success = True  # Acknowledge receipt of the goal

        # Make the timer trigger immediately to process new goal
        self.timer.reset()
        self.timer_callback()

        # Tell the lights to show nav is running
        msg = Int8()
        msg.data = self.nav_activate_light_code
        self.lights_publisher.publish(msg)

        return response

    def convert_lat_lon_to_pose(self, latitude, longitude, altitude, orientation):
        """Converts a latitude, longitude, altitude, and orientation to a PoseStamped in the map frame."""
        req = FromLL.Request()
        req.ll_point.longitude = longitude
        req.ll_point.latitude = latitude
        req.ll_point.altitude = altitude

        self.get_logger().info(
            f"Requesting map pose for: Long={req.ll_point.longitude:.8f}, Lat={req.ll_point.latitude:.8f}, Alt={req.ll_point.altitude:.8f}"
        )

        try:
            event = Event()

            def done_callback(future):
                nonlocal event
                event.set()

            future = self.localizer_client.call_async(req)
            future.add_done_callback(done_callback)
            event.wait()

            result = future.result()
            if result:
                target_pose = PoseStamped()
                target_pose.header.frame_id = "map"
                target_pose.header.stamp = self.get_clock().now().to_msg()
                target_pose.pose.position = result.map_point
                target_pose.pose.orientation = orientation
                self.get_logger().info(
                    f"Converted to map frame: x={target_pose.pose.position.x:.2f}, y={target_pose.pose.position.y:.2f}"
                )

                # hack for indoor testing
                # target_pose.pose.position.x = 50.0
                # target_pose.pose.position.y = 0.0
                # target_pose.pose.position.z = 0.0

                return target_pose
            else:
                self.get_logger().error("Failed to convert lat/lon to map pose")
                return None
        except Exception as e:
            self.get_logger().error(f"Error during lat/lon conversion: {e}")
            return None

    def euclidean_distance(self, pose1: PoseStamped, pose2: PoseStamped):
        """Calculates the Euclidean distance between two poses (ignoring z)."""
        if pose1 is None or pose2 is None:
            return float("inf")
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def calculate_intermediate_goal(
        self, current_pose: PoseStamped, final_pose: PoseStamped, distance: float
    ):
        """Calculates an intermediate goal point a certain distance along the path to the final goal."""
        if current_pose is None or final_pose is None:
            return None

        dx = final_pose.pose.position.x - current_pose.pose.position.x
        dy = final_pose.pose.position.y - current_pose.pose.position.y
        total_distance = math.sqrt(dx * dx + dy * dy)

        if total_distance <= distance:
            return final_pose  # Already within the desired distance

        ratio = distance / total_distance
        intermediate_x = current_pose.pose.position.x + dx * ratio
        intermediate_y = current_pose.pose.position.y + dy * ratio

        intermediate_goal = PoseStamped()
        intermediate_goal.header.frame_id = "map"
        intermediate_goal.header.stamp = self.get_clock().now().to_msg()
        intermediate_goal.pose.position.x = intermediate_x
        intermediate_goal.pose.position.y = intermediate_y
        intermediate_goal.pose.orientation = final_pose.pose.orientation
        return intermediate_goal

    def reset(self):
        self.goal_handle = None
        self.intermediate_goal_handle = None
        self.mission_state = MissionState.DO_NOTHING

    def timer_callback(self):
        """
        Timer callback handles all logic, state changes, intermediate goals, aruco naving
        """
        # self.get_logger().info("TIMER")
        if self.mission_state == MissionState.DO_NOTHING:
            return

        elif self.mission_state == MissionState.NAV_TO_INTERMEDIATE_GOAL:
            if self.final_lat_lon is None:
                self.get_logger().warn(
                    "Error in NAV_TO_INTERMEDIATE_GOAL: lat lon is None"
                )
                return
            elif self.current_robot_pose is None:
                self.get_logger().warn(
                    "Error in NAV_TO_INTERMEDIATE_GOAL: current robot pose is not available"
                )
                return

            # Calculate the final pose
            final_pose = self.convert_lat_lon_to_pose(*self.final_lat_lon)
            if final_pose is None:
                self.get_logger().warn(
                    "Error in NAV_TO_INTERMEDIATE_GOAL: Failed to convert lat lon to pose"
                )
                return

            distance_to_final = self.euclidean_distance(
                self.current_robot_pose, final_pose
            )
            if distance_to_final < self.incremental_distance:
                # Final pose is close, go to final pose
                self.mission_state = MissionState.NAV_TO_FINAL_GOAL

                self.get_logger().info(
                    f"Going to final goal: {final_pose.pose.position}"
                )
                self.intermediate_goal_publisher.publish(final_pose)
                self.goal_handle = self.navigator.goToPose(final_pose)
                self.get_logger().info(
                    f"Called goToPose for final pose with {final_pose.pose.position}"
                )
                self.get_logger().info(
                    f"~~~NAV_TO_INTERMEDIATE_GOAL -> NAV_TO_FINAL_GOAL. ({distance_to_final:.2f} meters to lat lon)"
                )
                return

            # Final pose is too far away, plan an intermediate goal closer
            intermediate_goal = self.calculate_intermediate_goal(
                self.current_robot_pose, final_pose, self.incremental_distance
            )
            self.get_logger().info(f"Final goal is {distance_to_final:.2f} meters away")
            if intermediate_goal is None:
                self.get_logger().info(
                    "Error in NAV_TO_INTERMEDIATE_GOAL: Failed to calculate intermediate goal"
                )
                return

            # Cancel current task and replan
            # self.get_logger().info(f"Cancelling task")
            # self.navigator.cancelTask()
            # self.get_logger().info(f"Task Cancelled")

            self.intermediate_goal_publisher.publish(intermediate_goal)
            self.get_logger().info(
                f"Publishing intermediate goal: x={intermediate_goal.pose.position.x:.2f}, y={intermediate_goal.pose.position.y:.2f}"
            )

            # if self.intermediate_goal_handle is None:
            self.get_logger().info(
                f"Calling goToPose for intermediate goal {intermediate_goal}"
            )
            self.intermediate_goal_handle = self.navigator.goToPose(intermediate_goal)
            self.get_logger().info(
                f"Called goToPose for intermediate goal. GoalHandle: {self.intermediate_goal_handle}"
            )

            # self.get_logger().info("Calling self.navigator.isTaskComplete()")
            if self.navigator.isTaskComplete():
                self.get_logger().warn(
                    "Error in NAV_TO_INTERMEDIATE_GOAL: Nav reported it completed going to intermediate goal"
                )
            # self.get_logger().info("Finishing isTaskComplete")
            return

        elif self.mission_state == MissionState.NAV_TO_FINAL_GOAL:
            if not self.lookForAruco:
                if self.goal_handle is None:
                    self.get_logger().warn(
                        "Error in NAV_TO_FINAL_GOAL: No goal handle available"
                    )
                    return

                if self.navigator.isTaskComplete():
                    self.get_logger().info("Final goal complete!")

                    msg = Int8()
                    msg.data = self.nav_completed_light_code
                    self.lights_publisher.publish(msg)

                    result = self.navigator.getResult()
                    if result == TaskResult.SUCCEEDED:
                        self.get_logger().info("Final goal succeeded!")
                    elif result == TaskResult.CANCELED:
                        self.get_logger().info("Final goal was canceled!")
                    elif result == TaskResult.FAILED:
                        self.get_logger().error("Final goal failed!")

                    self.reset()


def main(args=None):
    rclpy.init(args=args)
    node = IncrementalGpsCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
