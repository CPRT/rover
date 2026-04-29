#!/usr/bin/env python3

import os
import threading
from collections import Counter
from enum import Enum, auto
from threading import Event
from typing import List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import Int8, Bool, Int32MultiArray
from std_srvs.srv import Trigger
import tf2_ros

from action_msgs.msg import GoalInfo
from action_msgs.srv import CancelGoal
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from robot_localization.srv import FromLL
from interfaces.srv import NavToGPSGeopose

# Import custom search utilities
try:
    from nav_commanders.search_pattern_utils import generate_search_pattern
except ImportError:
    from search_pattern_utils import generate_search_pattern


class MissionState(Enum):
    DO_NOTHING = auto()
    NAV_TO_GPS = auto()
    EXECUTE_SEARCH = auto()
    FOUND_TARGET = auto()


class UnifiedNavCommander(Node):
    """Unified GPS and search commander for ROS 2 Humble."""

    def __init__(self):
        super().__init__("cprt_commander")
        self.navigator = BasicNavigator("cprt_commander")
        self.mission_state = MissionState.DO_NOTHING

        # --- Parameters ---
        self.declare_parameter("type", "gps")
        self.mission_type = (
            self.get_parameter("type")
            .get_parameter_value()
            .string_value.strip()
            .lower()
        )

        self.declare_parameter("frequency", 5.0)  # Hz
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
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

        self.declare_parameter("nav_bt_file", "bt_swerve_dynamic_replanning.xml")
        self.declare_parameter("search_bt_file", "bt_swerve_search_tree.xml")
        self.nav_bt_file = (
            self.get_parameter("nav_bt_file").get_parameter_value().string_value
        )
        self.search_bt_file = (
            self.get_parameter("search_bt_file").get_parameter_value().string_value
        )

        # Dynamically resolve BT paths
        nav_pkg_share = get_package_share_directory("navigation")
        self.nav_bt_path = os.path.join(
            nav_pkg_share, "behavior_trees", self.nav_bt_file
        )
        self.search_bt_path = os.path.join(
            nav_pkg_share, "behavior_trees", self.search_bt_file
        )

        # --- Mission Configurations ---
        # Maps the parameter to the correct YAML string and defines the detection style
        self.mission_configs = {
            "gps": {
                "search": False,
                "pattern": None,
                "topic": None,
                "is_aruco": False,
                "start_at_current_pose": False,
            },
            "aruco10m": {
                "search": True,
                "pattern": "spiral_10m",
                "topic": "/vision/aruco_detected",
                "is_aruco": True,
                "start_at_current_pose": False,
            },
            "aruco20m": {
                "search": True,
                "pattern": "spiral_20m",
                "topic": "/vision/aruco_detected",
                "is_aruco": True,
                "start_at_current_pose": False,
            },
            "mallet": {
                "search": True,
                "pattern": "spiral_5m",
                "topic": "/vision/mallet_detected",
                "is_aruco": False,
                "start_at_current_pose": False,
            },
            "pick": {
                "search": True,
                "pattern": "spiral_5m",
                "topic": "/vision/pick_detected",
                "is_aruco": False,
                "start_at_current_pose": False,
            },
            "bottle": {
                "search": True,
                "pattern": "spiral_10m",
                "topic": "/vision/bottle_detected",
                "is_aruco": False,
                "start_at_current_pose": False,
            },
            "indoor_spiral": {
                "search": True,
                "pattern": "spiral_20m",
                "topic": None,
                "is_aruco": False,
                "start_at_current_pose": True,
            },
        }

        if self.mission_type not in self.mission_configs:
            self.get_logger().error(
                f"Invalid mission type: {self.mission_type}. Defaulting to 'gps'."
            )
            self.config = self.mission_configs["gps"]
        else:
            self.config = self.mission_configs[self.mission_type]

        # --- Constants & QoS ---
        self.nav_fix_service_name = "fromLL"
        self.geopose_service_name = "commander/nav_to_gps_geopose"
        self.cancel_nav_service_name = "commander/cancel_nav"
        self.lights_topic = "/light"
        self.intended_path_topic = "/intended_search_path"

        self.nav_to_pose_cancel_service = "navigate_to_pose/_action/cancel_goal"
        self.nav_through_poses_cancel_service = (
            "navigate_through_poses/_action/cancel_goal"
        )

        self.nav_activate_light_code = 1
        self.nav_cancelled_light_code = 2
        self.nav_completed_light_code = 3

        self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.path_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        # --- Callback Groups ---
        self.localizer_cb_group = MutuallyExclusiveCallbackGroup()
        self.goal_cb_group = MutuallyExclusiveCallbackGroup()
        self.cancel_cb_group = MutuallyExclusiveCallbackGroup()
        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.lights_cb_group = MutuallyExclusiveCallbackGroup()
        self.detection_cb_group = ReentrantCallbackGroup()

        # --- Interfaces ---
        self.localizer_client = self.create_client(
            FromLL, self.nav_fix_service_name, callback_group=self.localizer_cb_group
        )
        self.geopose_service = self.create_service(
            NavToGPSGeopose,
            self.geopose_service_name,
            self.geopose_server,
            callback_group=self.goal_cb_group,
        )
        self.cancel_nav_service = self.create_service(
            Trigger,
            self.cancel_nav_service_name,
            self.cancel_nav_server,
            callback_group=self.cancel_cb_group,
        )

        self.nav_to_pose_cancel_client = self.create_client(
            CancelGoal, self.nav_to_pose_cancel_service
        )
        self.nav_through_poses_cancel_client = self.create_client(
            CancelGoal, self.nav_through_poses_cancel_service
        )

        self.lights_publisher = self.create_publisher(
            Int8,
            self.lights_topic,
            qos_profile=self.qos_profile,
            callback_group=self.lights_cb_group,
        )
        self.path_publisher = self.create_publisher(
            Path, self.intended_path_topic, qos_profile=self.path_qos
        )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Dynamic Detection Subscription ---
        if self.config["search"] and self.config["topic"]:
            if self.config["is_aruco"]:
                self.detection_subscription = self.create_subscription(
                    Int32MultiArray,
                    self.config["topic"],
                    self.aruco_callback,
                    qos_profile_sensor_data,
                    callback_group=self.detection_cb_group,
                )
            else:
                self.detection_subscription = self.create_subscription(
                    Bool,
                    self.config["topic"],
                    self.custom_object_callback,
                    qos_profile_sensor_data,
                    callback_group=self.detection_cb_group,
                )

        # --- State Variables ---
        self.current_goal_request: Optional[Tuple[float, float, float, object]] = None
        self.current_goal_pose: Optional[PoseStamped] = None
        self.goal_handle = None
        self.search_handle = None
        self.search_poses: List[PoseStamped] = []
        self.detections_buffer: List[Tuple[float, int]] = []
        self.stop_triggered_id: Optional[str] = None

        self.timer = self.create_timer(
            1.0 / self.frequency,
            self.timer_callback,
            callback_group=self.timer_cb_group,
        )

        # --- Init Checks ---
        if not self.config.get("start_at_current_pose"):
            self.get_logger().info(f"Waiting for {self.nav_fix_service_name}...")
            while not self.localizer_client.wait_for_service(timeout_sec=2.0):
                pass

        self.get_logger().info("Waiting for Nav2...")
        self.navigator.waitUntilNav2Active(localizer="controller_server")
        self.get_logger().info(
            f"Commander Ready. Loaded type '{self.mission_type}' with config: {self.config}"
        )

    # --- Helper Functions ---

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

    def reset_state_variables(self) -> None:
        self.goal_handle = None
        self.search_handle = None
        self.current_goal_request = None
        self.current_goal_pose = None
        self.search_poses = []
        self.detections_buffer = []
        self.stop_triggered_id = None
        self.mission_state = MissionState.DO_NOTHING

    def _cancel_all_nav_goals(self) -> None:
        self.get_logger().info("Canceling any active Nav2 goals...")
        for client, label in (
            (self.nav_to_pose_cancel_client, "navigate_to_pose"),
            (self.nav_through_poses_cancel_client, "navigate_through_poses"),
        ):
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(f"Cancel service not available for {label}.")
                continue

            request = CancelGoal.Request()
            request.goal_info = GoalInfo()

            # --- FIXED BLOCKING PATTERN ---
            event = Event()

            def done_callback(future):
                event.set()

            future = client.call_async(request)
            future.add_done_callback(done_callback)

            # Wait for up to 1.0 seconds without spinning the node
            event_set = event.wait(timeout=1.0)

            if not event_set:
                self.get_logger().warn(f"Cancel request timed out for {label}.")
                continue

            result = future.result()
            # ------------------------------

            if result is None:
                self.get_logger().warn(f"Cancel request failed for {label}.")
                continue

            if result.return_code not in (
                CancelGoal.Response.ERROR_NONE,
                CancelGoal.Response.ERROR_GOAL_TERMINATED,
                CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID,
            ):
                self.get_logger().warn(
                    f"Cancel request rejected for {label} (code {result.return_code})."
                )
        self.get_logger().info("Cancel requests finished.")

    def publish_search_path(self, poses):
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = poses
        self.path_publisher.publish(path_msg)

    def get_current_pose(self) -> Optional[PoseStamped]:
        try:
            tf = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=0.5)
            )
        except Exception as exc:
            self.get_logger().warn(f"TF lookup failed for map->base_link: {exc}")
            return None

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = tf.transform.translation.x
        pose.pose.position.y = tf.transform.translation.y
        pose.pose.position.z = tf.transform.translation.z
        pose.pose.orientation = tf.transform.rotation
        return pose

    # --- Callbacks & Services ---

    def geopose_server(
        self, request: NavToGPSGeopose.Request, response: NavToGPSGeopose.Response
    ) -> NavToGPSGeopose.Response:
        self.get_logger().info(f"Received mission request: {request.goal}")
        self._cancel_all_nav_goals()
        self.reset_state_variables()
        if self.config.get("start_at_current_pose"):
            self.get_logger().info("Fetching current pose for indoor spiral...")
            self.current_goal_pose = self.get_current_pose()
            if self.current_goal_pose is None:
                response.success = False
                self._fail_mission("Failed to get current pose for indoor spiral.")
                return response
            self.get_logger().info("Current pose acquired. Starting search phase.")
            self.mission_state = MissionState.EXECUTE_SEARCH
        else:
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

    def cancel_nav_server(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        self.get_logger().info("Received request to cancel mission.")
        self.navigator.cancelTask()
        self._fail_mission("Mission cancelled by operator.")
        response.success = True
        return response

    def _trigger_target_found(self, target_id: str):
        """Halts navigation and completes the mission."""
        self.get_logger().info(
            f"Target '{target_id}' detected! Stopping navigation/search immediately."
        )
        self.stop_triggered_id = target_id
        self.mission_state = MissionState.FOUND_TARGET
        self.navigator.cancelTask()

    def aruco_callback(self, msg: Int32MultiArray) -> None:
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
            if count >= self.aruco_min_detections:
                self._trigger_target_found(f"Aruco ID {tag_id}")
                return

    def custom_object_callback(self, msg: Bool) -> None:
        if self.mission_state not in (
            MissionState.NAV_TO_GPS,
            MissionState.EXECUTE_SEARCH,
        ):
            return

        if msg.data:
            self._trigger_target_found(self.mission_type)

    def convert_lat_lon_to_pose(self, latitude, longitude, altitude, orientation):
        req = FromLL.Request()
        req.ll_point.latitude, req.ll_point.longitude, req.ll_point.altitude = (
            latitude,
            longitude,
            altitude,
        )
        try:
            event = Event()

            def done_callback(future):
                event.set()

            future = self.localizer_client.call_async(req)
            future.add_done_callback(done_callback)
            event.wait(timeout=5.0)

            result = future.result()
            if not result:
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

    # --- Main State Machine ---

    def timer_callback(self) -> None:
        # self.get_logger().info(
        #     f"Timer tick: mission_state={self.mission_state.name}"
        # )
        if self.mission_state in [MissionState.DO_NOTHING, MissionState.FOUND_TARGET]:
            return

        if self.current_goal_pose is None and self.current_goal_request is not None:
            self.current_goal_pose = self.convert_lat_lon_to_pose(
                *self.current_goal_request
            )
            if self.current_goal_pose is None:
                self._fail_mission("Failed to convert GPS coordinate to Map frame.")
                return

        # 1. GPS Phase
        if self.mission_state == MissionState.NAV_TO_GPS:
            if self.goal_handle is None:
                self.get_logger().info(
                    f"Navigating to GPS goal. BT: {self.nav_bt_file}"
                )
                self.goal_handle = "PENDING_DISPATCH"

                def dispatch_gps():
                    self.navigator.goToPose(
                        self.current_goal_pose, behavior_tree=self.nav_bt_path
                    )
                    self.goal_handle = True

                threading.Thread(target=dispatch_gps, daemon=True).start()
                return

            if self.goal_handle == "PENDING_DISPATCH":
                return

            if not self.navigator.isTaskComplete():
                return

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                if not self.config["search"]:
                    self._finish_mission("GPS goal reached. Mission complete.")
                else:
                    self.get_logger().info(
                        "GPS goal reached. Transitioning to Search phase."
                    )
                    self.mission_state = MissionState.EXECUTE_SEARCH
                    self.goal_handle = None
                return

            if result == TaskResult.CANCELED:
                if self.mission_state == MissionState.FOUND_TARGET:
                    self._finish_mission(
                        f"Target found early! Stopped at {self.stop_triggered_id}."
                    )
                else:
                    self._fail_mission("GPS navigation canceled unexpectedly.")
                return

            self._fail_mission("GPS navigation failed.")

        # 2. Search Phase
        elif self.mission_state == MissionState.EXECUTE_SEARCH:
            if self.search_handle is None:
                self.get_logger().info("Starting search pattern generation...")
                self.search_poses = generate_search_pattern(
                    pattern=self.config["pattern"],
                    center_x=self.current_goal_pose.pose.position.x,
                    center_y=self.current_goal_pose.pose.position.y,
                )

                if not self.search_poses:
                    self._fail_mission("Search pattern generator returned empty list.")
                    return

                self.publish_search_path(self.search_poses)
                self.get_logger().info(
                    f"Executing '{self.config['pattern']}' pattern ({len(self.search_poses)} poses). BT: {self.search_bt_file}"
                )

                self.search_handle = "PENDING_DISPATCH"

                def dispatch_search():
                    self.navigator.goThroughPoses(
                        self.search_poses, behavior_tree=self.search_bt_path
                    )
                    self.search_handle = True

                threading.Thread(target=dispatch_search, daemon=True).start()
                return

            if self.search_handle == "PENDING_DISPATCH":
                return

            if not self.navigator.isTaskComplete():
                return

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self._finish_mission(
                    "Search pattern fully traversed without finding object."
                )
                return

            if result == TaskResult.CANCELED:
                if self.mission_state == MissionState.FOUND_TARGET:
                    self._finish_mission(
                        f"Search successful! Stopped at {self.stop_triggered_id}."
                    )
                else:
                    self._fail_mission("Search pattern canceled unexpectedly.")
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
