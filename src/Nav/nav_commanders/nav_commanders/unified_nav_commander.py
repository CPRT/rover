#!/usr/bin/env python3

from collections import Counter
from enum import Enum, auto
from threading import Event
from typing import List, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from std_msgs.msg import Int8, Int32MultiArray

from interfaces.srv import NavToGPSGeopose
from robot_localization.srv import FromLL

try:
    from .search_pattern_utils import generate_search_pattern
except ImportError:
    from search_pattern_utils import generate_search_pattern


class MissionState(Enum):
    DO_NOTHING = auto()
    NAV_TO_GPS = auto()
    EXECUTE_SEARCH = auto()


class UnifiedNavCommander(Node):
    """Unified GPS and search commander for ROS 2 Humble."""

    def __init__(self):
        super().__init__("unified_nav_commander")
        self.navigator = BasicNavigator("unified_nav_navigator")
        self.mission_state = MissionState.DO_NOTHING

        self.declare_parameter("type", "gps")
        self.mission_type = (
            self.get_parameter("type").get_parameter_value().string_value
        )
        self.mission_type = self.mission_type.strip().lower()

        self.declare_parameter("aruco_topic", "/aruco_detections")
        self.aruco_topic = (
            self.get_parameter("aruco_topic").get_parameter_value().string_value
        )

        self.declare_parameter("aruco_detection_window_sec", 5.0)
        self.aruco_detection_window_sec = (
            self.get_parameter("aruco_detection_window_sec")
            .get_parameter_value()
            .double_value
        )

        self.declare_parameter("aruco_min_detections", 5)
        self.aruco_min_detections = (
            self.get_parameter("aruco_min_detections")
            .get_parameter_value()
            .integer_value
        )

        self.declare_parameter("robot_pose_topic", "odometry/filtered/global")
        self.robot_pose_topic = (
            self.get_parameter("robot_pose_topic").get_parameter_value().string_value
        )

        self.declare_parameter("frequency", 1 / 5.0)
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )

        self.declare_parameter("gps_behavior_tree", "bt_swerve_dynamic_replanning.xml")
        self.gps_behavior_tree_filename = (
            self.get_parameter("gps_behavior_tree").get_parameter_value().string_value
        )

        self.declare_parameter("search_behavior_tree", "bt_swerve_search_tree.xml")
        self.search_behavior_tree_filename = (
            self.get_parameter("search_behavior_tree")
            .get_parameter_value()
            .string_value
        )

        self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.path_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        self.nav_fix_service_name = "fromLL"
        self.geopose_service_name = "commander/nav_to_gps_geopose"
        self.lights_topic = "/light"
        self.nav_activate_light_code = 1
        self.nav_completed_light_code = 3
        self.nav_cancelled_light_code = 2

        self.localizer_callback_group = MutuallyExclusiveCallbackGroup()
        self.goal_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.pose_callback_group = ReentrantCallbackGroup()
        self.aruco_callback_group = ReentrantCallbackGroup()
        self.lights_callback_group = MutuallyExclusiveCallbackGroup()

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

        self.robot_pose_subscription = self.create_subscription(
            Odometry,
            self.robot_pose_topic,
            self.robot_pose_callback,
            qos_profile_sensor_data,
            callback_group=self.pose_callback_group,
        )

        self.aruco_subscription = self.create_subscription(
            Int32MultiArray,
            self.aruco_topic,
            self.aruco_callback,
            qos_profile_sensor_data,
            callback_group=self.aruco_callback_group,
        )

        self.lights_publisher = self.create_publisher(
            Int8,
            self.lights_topic,
            qos_profile=self.qos_profile,
            callback_group=self.lights_callback_group,
        )

        self.current_robot_pose = None
        self.current_goal_request: Optional[Tuple[float, float, float, object]] = None
        self.current_goal_pose: Optional[PoseStamped] = None
        self.goal_handle = None
        self.search_handle = None
        self.search_poses: List[PoseStamped] = []
        self.detections_buffer: List[Tuple[float, int]] = []
        self.aruco_stop_triggered_id: Optional[int] = None

        self.mission_settings = self._mission_settings_for_type(self.mission_type)
        self.search_radius = self.mission_settings["search_radius"]
        self.stop_on_detection = self.mission_settings["stop_on_detection"]

        self.gps_behavior_tree_path = self._resolve_behavior_tree_path(
            self.gps_behavior_tree_filename
        )
        self.search_behavior_tree_path = self._resolve_behavior_tree_path(
            self.search_behavior_tree_filename
        )

        self.timer = self.create_timer(
            1.0 / self.frequency, self.timer_callback, callback_group=self.timer_group
        )

        self.get_logger().info(f"Mission type: {self.mission_type}")
        self.get_logger().info(f"GPS BT: {self.gps_behavior_tree_path}")
        self.get_logger().info(f"Search BT: {self.search_behavior_tree_path}")
        self.get_logger().info(f"Aruco topic: {self.aruco_topic}")

        self.get_logger().info(
            f"Waiting for {self.nav_fix_service_name} to be active..."
        )
        while not self.localizer_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Waiting for fromLL...")
        self.get_logger().info("fromLL is active")

        self.get_logger().info("Waiting for Nav2 to be active...")
        self.navigator.waitUntilNav2Active(localizer="controller_server")
        self.get_logger().info("Nav2 is active")

    def _mission_settings_for_type(self, mission_type: str) -> dict:
        settings = {
            "gps": {"search_radius": None, "stop_on_detection": False},
            "aruco10m": {"search_radius": 10.0, "stop_on_detection": True},
            "aruco20m": {"search_radius": 20.0, "stop_on_detection": True},
            "mallet": {"search_radius": 5.0, "stop_on_detection": True},
            "pick": {"search_radius": 5.0, "stop_on_detection": True},
            "bottle": {"search_radius": 10.0, "stop_on_detection": True},
        }
        if mission_type not in settings:
            raise ValueError(
                f"Unknown type '{mission_type}'. Expected one of: {', '.join(sorted(settings.keys()))}"
            )
        return settings[mission_type]

    def _resolve_behavior_tree_path(self, filename: str) -> str:
        if not filename:
            return ""

        path = filename.strip()
        if path.startswith("/"):
            return path

        navigation_share = get_package_share_directory("navigation")
        return f"{navigation_share}/behavior_trees/{path}"

    def _publish_light(self, code: int) -> None:
        msg = Int8()
        msg.data = code
        self.lights_publisher.publish(msg)

    def _finish_mission(self, message: str, light_code: int = 3) -> None:
        self.get_logger().info(message)
        self._publish_light(light_code)
        self.reset_state_variables()

    def _fail_mission(self, message: str) -> None:
        self.get_logger().error(message)
        self._publish_light(self.nav_cancelled_light_code)
        self.reset_state_variables()

    def _generate_search_poses(self, center_pose: PoseStamped) -> List[PoseStamped]:
        if self.search_radius is None:
            return []

        return generate_search_pattern(
            pattern="spiral",
            center_x=center_pose.pose.position.x,
            center_y=center_pose.pose.position.y,
            frame_id="map",
            search_radius=self.search_radius,
        )

    def _start_navigation_to_gps(self) -> None:
        if self.current_goal_pose is None:
            return

        if self.goal_handle is None:
            self.get_logger().info(
                f"Navigating directly to GPS goal: x={self.current_goal_pose.pose.position.x:.2f}, y={self.current_goal_pose.pose.position.y:.2f}"
            )
            self.goal_handle = self.navigator.goToPose(
                self.current_goal_pose,
                behavior_tree=self.gps_behavior_tree_path,
            )

    def _start_search(self) -> None:
        if self.current_goal_pose is None:
            return

        if not self.search_poses:
            self.search_poses = self._generate_search_poses(self.current_goal_pose)
            if not self.search_poses:
                self._fail_mission("Search pattern could not be generated.")
                return
            self.get_logger().info(
                f"Generated {len(self.search_poses)} search waypoints for radius {self.search_radius} m"
            )

        if self.search_handle is None:
            self.get_logger().info("Executing search pattern...")
            self.search_handle = self.navigator.goThroughPoses(
                self.search_poses,
                behavior_tree=self.search_behavior_tree_path,
            )

    def robot_pose_callback(self, msg: Odometry) -> None:
        self.current_robot_pose = msg.pose

    def aruco_callback(self, msg: Int32MultiArray) -> None:
        if not self.stop_on_detection:
            return

        if self.mission_state not in (
            MissionState.NAV_TO_GPS,
            MissionState.EXECUTE_SEARCH,
        ):
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        self.detections_buffer = [
            (timestamp, tag_id)
            for timestamp, tag_id in self.detections_buffer
            if (current_time - timestamp) <= self.aruco_detection_window_sec
        ]

        for detected_id in msg.data:
            self.detections_buffer.append((current_time, int(detected_id)))

        tag_counts = Counter(tag_id for _timestamp, tag_id in self.detections_buffer)
        for tag_id, count in tag_counts.items():
            if count < self.aruco_min_detections:
                continue

            self.get_logger().info(
                f"Aruco ID {tag_id} detected {count} times within {self.aruco_detection_window_sec} seconds. Stopping mission."
            )
            self.aruco_stop_triggered_id = tag_id
            self.navigator.cancelTask()
            self._finish_mission(
                f"Target detected, mission complete on Aruco ID {tag_id}."
            )
            return

    def geopose_server(
        self, request: NavToGPSGeopose.Request, response: NavToGPSGeopose.Response
    ) -> NavToGPSGeopose.Response:
        self.get_logger().info(f"Received mission request: {request.goal}")
        self.reset_state_variables()
        self.current_goal_request = (
            request.goal.position.latitude,
            request.goal.position.longitude,
            request.goal.position.altitude,
            request.goal.orientation,
        )
        self.mission_state = MissionState.NAV_TO_GPS
        response.success = True
        self.timer.reset()
        self._publish_light(self.nav_activate_light_code)
        return response

    def convert_lat_lon_to_pose(self, latitude, longitude, altitude, orientation):
        from robot_localization.srv import FromLL

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
                self.get_logger().error("fromLL service timeout")
                return None

            result = future.result()
            if not result:
                self.get_logger().error("fromLL returned no result")
                return None

            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.header.stamp = self.get_clock().now().to_msg()
            target_pose.pose.position = result.map_point
            target_pose.pose.orientation = orientation
            return target_pose
        except Exception as exc:
            self.get_logger().error(f"Error during lat/lon conversion: {exc}")
            return None

    def reset_state_variables(self) -> None:
        self.goal_handle = None
        self.search_handle = None
        self.current_goal_request = None
        self.current_goal_pose = None
        self.search_poses = []
        self.detections_buffer = []
        self.aruco_stop_triggered_id = None
        self.mission_state = MissionState.DO_NOTHING

    def timer_callback(self) -> None:
        if self.mission_state == MissionState.DO_NOTHING:
            return

        if self.current_goal_request is None:
            self._fail_mission("No GPS goal available for the current mission.")
            return

        if self.current_goal_pose is None:
            self.current_goal_pose = self.convert_lat_lon_to_pose(
                *self.current_goal_request
            )
            if self.current_goal_pose is None:
                return

        if self.mission_state == MissionState.NAV_TO_GPS:
            self._start_navigation_to_gps()
            if self.goal_handle is None:
                return

            if not self.navigator.isTaskComplete():
                return

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Reached GPS goal.")
                if self.search_radius is None:
                    self._finish_mission("GPS mission complete.")
                else:
                    self.mission_state = MissionState.EXECUTE_SEARCH
                    self.goal_handle = None
                    self.search_handle = None
                return

            if result == TaskResult.CANCELED:
                self._finish_mission(
                    "GPS navigation canceled.", light_code=self.nav_cancelled_light_code
                )
                return

            self._fail_mission("GPS navigation failed.")
            return

        if self.mission_state == MissionState.EXECUTE_SEARCH:
            self._start_search()
            if self.search_handle is None:
                return

            if not self.navigator.isTaskComplete():
                return

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self._finish_mission("Search pattern complete.")
                return

            if result == TaskResult.CANCELED:
                if self.aruco_stop_triggered_id is not None:
                    self._finish_mission(
                        f"Search stopped by Aruco ID {self.aruco_stop_triggered_id}."
                    )
                else:
                    self._finish_mission(
                        "Search pattern canceled.",
                        light_code=self.nav_cancelled_light_code,
                    )
                return

            self._fail_mission("Search pattern failed.")


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedNavCommander()
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
