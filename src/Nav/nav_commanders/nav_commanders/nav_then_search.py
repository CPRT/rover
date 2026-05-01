#!/usr/bin/env python3

import math
from enum import Enum, auto
from threading import Event
from collections import Counter

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int8, Int32MultiArray
from std_srvs.srv import Trigger

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from robot_localization.srv import FromLL
from interfaces.srv import NavToGPSGeopose


class MissionState(Enum):
    DO_NOTHING = auto()
    NAV_TO_START = auto()
    EXECUTE_SEARCH = auto()
    FOUND_TARGET = auto()
    NAV_CANCELLED = auto()


class SearchPatternCommander(Node):
    def __init__(self):
        super().__init__("search_pattern_commander")
        self.navigator = BasicNavigator("search_pattern_navigator")
        self.mission_state = MissionState.DO_NOTHING

        # --- Parameters ---
        self.declare_parameter("object_topic", "/aruco_detections")
        self.object_topic = (
            self.get_parameter("object_topic").get_parameter_value().string_value
        )

        self.declare_parameter("search_size_meters", 20.0)
        self.search_size = (
            self.get_parameter("search_size_meters").get_parameter_value().double_value
        )

        self.declare_parameter("lane_spacing_meters", 4.0)
        self.lane_spacing = (
            self.get_parameter("lane_spacing_meters").get_parameter_value().double_value
        )

        self.declare_parameter("frequency", 1 / 5.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )

        self.declare_parameter("robot_pose_topic", "odometry/filtered/global")
        self.robot_pose_topic = (
            self.get_parameter("robot_pose_topic").get_parameter_value().string_value
        )

        self.declare_parameter("detection_window_sec", 5.0)
        self.detection_window_sec = (
            self.get_parameter("detection_window_sec")
            .get_parameter_value()
            .double_value
        )

        self.declare_parameter("min_detections", 5)
        self.min_detections = (
            self.get_parameter("min_detections").get_parameter_value().integer_value
        )

        self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        # Transient Local QoS for the Path so it stays visible in RViz
        self.path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # --- Topic Names and Service Names ---
        self.nav_fix_service_name = "fromLL"
        self.geopose_service_name = "commander/nav_to_gps_geopose"
        self.cancel_nav_service_name = "commander/cancel_nav"
        self.lights_topic = "/light"
        self.intended_path_topic = "/intended_search_path"

        self.nav_activate_light_code = 1
        self.nav_completed_light_code = 3
        self.nav_cancelled_light_code = 2

        # --- Callback Groups ---
        self.localizer_callback_group = MutuallyExclusiveCallbackGroup()
        self.pose_callback_group = ReentrantCallbackGroup()
        self.goal_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.lights_callback_group = MutuallyExclusiveCallbackGroup()
        self.detection_callback_group = ReentrantCallbackGroup()
        self.cancel_service_callback_group = MutuallyExclusiveCallbackGroup()

        # --- Clients, Services, Subscriptions, Publishers ---
        self.localizer_client = self.create_client(
            FromLL,
            self.nav_fix_service_name,
            callback_group=self.localizer_callback_group,
        )
        self.geopose_service = self.create_service(
            NavToGPSGeopose,
            self.geopose_service_name,
            self.geopose_server,
            callback_group=self.goal_callback_group,
        )
        self.cancel_nav_service = self.create_service(
            Trigger,
            self.cancel_nav_service_name,
            self.cancel_nav_server,
            callback_group=self.cancel_service_callback_group,
        )
        self.robot_pose_subscription = self.create_subscription(
            Odometry,
            self.robot_pose_topic,
            self.robot_pose_callback,
            qos_profile_sensor_data,
            callback_group=self.pose_callback_group,
        )
        self.lights_publisher = self.create_publisher(
            Int8,
            self.lights_topic,
            qos_profile=self.qos_profile,
            callback_group=self.lights_callback_group,
        )

        # New Publisher for RViz full path visualization
        self.path_publisher = self.create_publisher(
            Path, self.intended_path_topic, qos_profile=self.path_qos
        )

        # --- Object Detection ---
        self.detections_buffer = []
        self.detection_subscription = self.create_subscription(
            Int32MultiArray,
            self.object_topic,
            self.detection_callback,
            qos_profile_sensor_data,
            callback_group=self.detection_callback_group,
        )

        # --- State Variables ---
        self.final_lat_lon = None
        self.current_robot_pose = None
        self.goal_handle = None
        self.stop_triggered_id = None
        self.search_poses = []

        # --- Timer ---
        self.timer = self.create_timer(
            1.0 / self.frequency, self.timer_callback, callback_group=self.timer_group
        )

        # --- Initialization Checks ---
        self.get_logger().info(
            f"Waiting for {self.nav_fix_service_name} to be active..."
        )
        while not self.localizer_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting...")
        self.get_logger().info(f"{self.nav_fix_service_name} is active")

        self.get_logger().info("Waiting for Nav2 to be active...")
        self.navigator.waitUntilNav2Active(localizer="controller_server")
        self.get_logger().info("Nav2 is active. Ready to generate search patterns.")

    def robot_pose_callback(self, msg: Odometry):
        self.current_robot_pose = msg.pose

    def detection_callback(self, msg: Int32MultiArray):
        if self.mission_state not in [
            MissionState.NAV_TO_START,
            MissionState.EXECUTE_SEARCH,
        ]:
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        self.detections_buffer = [
            (ts, tag_id)
            for ts, tag_id in self.detections_buffer
            if (current_time - ts) <= self.detection_window_sec
        ]

        for detected_id in msg.data:
            self.detections_buffer.append((current_time, detected_id))

        tag_ids_in_window = [tag_id for ts, tag_id in self.detections_buffer]
        tag_counts = Counter(tag_ids_in_window)

        for tag_id, count in tag_counts.items():
            if count >= self.min_detections:
                self.get_logger().info(f"Object ID {tag_id} detected! Stopping search.")
                self.mission_state = MissionState.FOUND_TARGET
                self.stop_triggered_id = tag_id
                self.navigator.cancelTask()
                self.reset_state_variables()

                light_msg = Int8()
                light_msg.data = self.nav_completed_light_code
                self.lights_publisher.publish(light_msg)
                return

    def geopose_server(
        self, request: NavToGPSGeopose.Request, response: NavToGPSGeopose.Response
    ) -> NavToGPSGeopose.Response:
        self.get_logger().info(f"Received new GPS search zone: {request.goal}")
        self.reset_state_variables()
        self.final_lat_lon = (
            request.goal.position.latitude,
            request.goal.position.longitude,
            request.goal.position.altitude,
            request.goal.orientation,
        )
        self.mission_state = MissionState.NAV_TO_START
        response.success = True
        self.timer.reset()

        msg = Int8()
        msg.data = self.nav_activate_light_code
        self.lights_publisher.publish(msg)
        return response

    def cancel_nav_server(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.get_logger().info("Received request to cancel search.")
        self.navigator.cancelTask()
        self.reset_state_variables()
        self.mission_state = MissionState.NAV_CANCELLED

        msg = Int8()
        msg.data = self.nav_cancelled_light_code
        self.lights_publisher.publish(msg)

        response.success = True
        response.message = "Search cancelled by user."
        return response

    def convert_lat_lon_to_pose(self, latitude, longitude, altitude, orientation):
        req = FromLL.Request()
        req.ll_point.longitude = longitude
        req.ll_point.latitude = latitude
        req.ll_point.altitude = altitude

        try:
            event = Event()

            def done_callback(future):
                nonlocal event
                event.set()

            future = self.localizer_client.call_async(req)
            future.add_done_callback(done_callback)
            event.wait(timeout=5.0)

            if not future.done():
                self.get_logger().error("fromLL service timeout.")
                return None

            result = future.result()
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position = result.map_point
            target_pose.pose.orientation = orientation
            return target_pose
        except Exception as e:
            self.get_logger().error(f"Error during lat/lon conversion: {e}")
            return None

    def yaw_to_quaternion(self, yaw):
        q = Quaternion()
        q.x, q.y = 0.0, 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

    def publish_intended_path(self, poses):
        """Publishes the full array of search poses to RViz."""
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = poses
        self.path_publisher.publish(path_msg)
        self.get_logger().info(
            f"Published intended search path to {self.intended_path_topic}"
        )

    def generate_search_pattern(self, center_pose: PoseStamped):
        poses = []
        c_x = center_pose.pose.position.x
        c_y = center_pose.pose.position.y
        half_size = self.search_size / 2.0

        start_x = c_x - half_size
        start_y = c_y - half_size
        num_lanes = int(self.search_size / self.lane_spacing) + 1

        for i in range(num_lanes):
            current_y = start_y + (i * self.lane_spacing)
            if i % 2 == 0:
                x_start, x_end, yaw = start_x, start_x + self.search_size, 0.0
            else:
                x_start, x_end, yaw = start_x + self.search_size, start_x, math.pi

            pose_start = PoseStamped()
            pose_start.header.frame_id = "map"
            pose_start.pose.position.x = x_start
            pose_start.pose.position.y = current_y
            pose_start.pose.orientation = self.yaw_to_quaternion(yaw)
            poses.append(pose_start)

            pose_end = PoseStamped()
            pose_end.header.frame_id = "map"
            pose_end.pose.position.x = x_end
            pose_end.pose.position.y = current_y
            pose_end.pose.orientation = self.yaw_to_quaternion(yaw)
            poses.append(pose_end)

        self.get_logger().info(
            f"Generated {len(poses)} waypoints for {self.search_size}x{self.search_size}m search area."
        )

        # Immediately publish to RViz so it's clear what the rover is attempting
        self.publish_intended_path(poses)
        return poses

    def reset_state_variables(self):
        self.goal_handle = None
        self.final_lat_lon = None
        self.detections_buffer = []
        self.stop_triggered_id = None
        self.search_poses = []

        # Clear the RViz path when resetting
        empty_path = Path()
        empty_path.header.frame_id = "map"
        self.path_publisher.publish(empty_path)

    def handle_task_result(self, success_state):
        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            if success_state == MissionState.EXECUTE_SEARCH:
                self.get_logger().info(
                    "Arrived at start point. Beginning search pattern!"
                )
                self.mission_state = MissionState.EXECUTE_SEARCH
                self.goal_handle = None
            else:
                self.get_logger().info(
                    "Search pattern fully completed without finding object."
                )
                self.mission_state = MissionState.DO_NOTHING
                self.reset_state_variables()
                msg = Int8()
                msg.data = self.nav_completed_light_code
                self.lights_publisher.publish(msg)

        elif result == TaskResult.CANCELED:
            if self.stop_triggered_id is not None:
                self.get_logger().info(
                    f"Mission cancelled by target detection (ID: {self.stop_triggered_id})."
                )
            else:
                self.get_logger().info("Mission canceled by user.")
        elif result == TaskResult.FAILED:
            self.get_logger().error("Navigation task failed!")
            self.mission_state = MissionState.DO_NOTHING
            self.reset_state_variables()

    def timer_callback(self):
        if self.mission_state == MissionState.NAV_TO_START:
            if not self.search_poses:
                center_pose = self.convert_lat_lon_to_pose(*self.final_lat_lon)
                if not center_pose:
                    return

                self.search_poses = self.generate_search_pattern(center_pose)

                self.get_logger().info("Navigating to the search start point...")
                self.goal_handle = self.navigator.goToPose(self.search_poses[0])
                return

            if self.navigator.isTaskComplete():
                self.handle_task_result(success_state=MissionState.EXECUTE_SEARCH)

        elif self.mission_state == MissionState.EXECUTE_SEARCH:
            if self.goal_handle is None:
                self.get_logger().info("Executing goThroughPoses for search grid...")
                self.goal_handle = self.navigator.goThroughPoses(self.search_poses[1:])
                return

            if self.navigator.isTaskComplete():
                self.handle_task_result(success_state=MissionState.DO_NOTHING)


def main(args=None):
    rclpy.init(args=args)
    node = SearchPatternCommander()
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
