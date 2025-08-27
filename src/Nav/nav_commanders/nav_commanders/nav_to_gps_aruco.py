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
from std_msgs.msg import Int8, Int32MultiArray
from std_srvs.srv import Trigger
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
import math
from enum import Enum, auto
from threading import Event
from collections import Counter


class MissionState(Enum):
    """
    Defines the possible states of the robot's navigation mission.
    """

    DO_NOTHING = auto()
    NAV_TO_GOAL = auto()  # Robot is actively navigating to a GPS goal.
    FOUND_ARUCO_MARKER = auto()  # Robot stopped navigation due to Aruco tag detection.
    NAV_CANCELLED = auto()  # Navigation was cancelled by a service call.


class IncrementalGpsCommander(Node):
    """
    ROS2 node to navigate to a GPS goal, with integrated Aruco tag detection
    for stopping autonomy and a service for external cancellation.
    """

    def __init__(self):
        super().__init__("incremental_gps_commander")
        self.navigator = BasicNavigator("incremental_gps_navigator")
        self.mission_state = MissionState.DO_NOTHING

        # --- Parameters ---
        self.declare_parameter("aruco_topic", "/aruco_detections")
        self.aruco_topic = (
            self.get_parameter("aruco_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Aruco topic set to: {self.aruco_topic}")

        self.declare_parameter(
            "frequency", 1 / 5.0
        )  # Frequency in Hz (default: 5 seconds per cycle)
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

        # New parameters for Aruco detection window and minimum detections
        self.declare_parameter("aruco_detection_window_sec", 5.0)
        self.aruco_detection_window_sec = (
            self.get_parameter("aruco_detection_window_sec")
            .get_parameter_value()
            .double_value
        )
        self.get_logger().info(
            f"Aruco detection window: {self.aruco_detection_window_sec} seconds"
        )

        self.declare_parameter("aruco_min_detections", 5)
        self.aruco_min_detections = (
            self.get_parameter("aruco_min_detections")
            .get_parameter_value()
            .integer_value
        )
        self.get_logger().info(f"Aruco minimum detections: {self.aruco_min_detections}")

        self.qos_profile = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)

        # --- Topic Names and Service Names ---
        self.nav_fix_service_name = (
            "fromLL"  # Service for converting Lat/Lon to map coordinates
        )
        self.geopose_service_name = (
            "commander/nav_to_gps_geopose"  # Service to receive new GPS goals
        )
        self.cancel_nav_service_name = (
            "commander/cancel_nav"  # Service to cancel active navigation
        )
        self.lights_topic = "/light"  # Topic to publish light codes for robot status

        # Light codes for different navigation states
        self.nav_activate_light_code = 1
        self.nav_completed_light_code = 3
        self.nav_cancelled_light_code = 2

        # --- Callback Groups ---
        # Used to manage concurrency for different types of callbacks
        self.localizer_callback_group = MutuallyExclusiveCallbackGroup()
        self.pose_callback_group = ReentrantCallbackGroup()
        self.goal_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.lights_callback_group = MutuallyExclusiveCallbackGroup()
        self.aruco_callback_group = ReentrantCallbackGroup()
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

        # --- Aruco Tag Specifics (always active) ---
        # Stores (timestamp, detected_id) tuples for all detected tags within the defined window.
        self.aruco_detections_buffer = []
        self.aruco_subscription = self.create_subscription(
            Int32MultiArray,
            self.aruco_topic,
            self.aruco_callback,
            qos_profile_sensor_data,
            callback_group=self.aruco_callback_group,
        )
        self.get_logger().info(f"Subscribing to Aruco detections on {self.aruco_topic}")

        # --- State Variables ---
        self.final_lat_lon = None  # Stores the final GPS goal
        self.current_robot_pose = None  # Stores the robot's current odometry pose
        self.goal_handle = None  # Handle for the active navigation goal
        self.aruco_stop_triggered_id = (
            None  # Stores the ID of the Aruco tag that triggered a stop
        )

        # --- Timer ---
        # The main timer for periodic checks and state management.
        self.timer = self.create_timer(
            1.0 / self.frequency, self.timer_callback, callback_group=self.timer_group
        )

        # --- Initialization Checks ---
        self.get_logger().info(
            "Nav fix service name: " + str(self.nav_fix_service_name)
        )
        self.get_logger().info(f"Waiting for {self.nav_fix_service_name} to be active")
        # Wait until the /fromLL service is available
        while not self.localizer_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                f"Service on {self.nav_fix_service_name} is not available, waiting again..."
            )
        self.get_logger().info(f"{self.nav_fix_service_name} is active")

        self.get_logger().info("Waiting for Nav2 to be active")
        # Wait until Nav2's controller server is active
        self.navigator.waitUntilNav2Active(localizer="controller_server")
        self.get_logger().info("Nav2 is active")

    def robot_pose_callback(self, msg: Odometry):
        """Callback to update the current robot pose from odometry."""
        self.current_robot_pose = msg.pose

    def aruco_callback(self, msg: Int32MultiArray):
        """
        Callback for Aruco tag detections.
        Tracks the frequency of all detected tags within a configurable sliding window.
        If any tag is detected a configurable number of times within this window, navigation is stopped.
        """
        # Only process Aruco detections if the robot is currently navigating to a goal.
        if self.mission_state != MissionState.NAV_TO_GOAL:
            return

        current_time = (
            self.get_clock().now().nanoseconds / 1e9
        )  # Current time in seconds

        # Filter out old detections from the buffer based on the configured window
        self.aruco_detections_buffer = [
            (ts, tag_id)
            for ts, tag_id in self.aruco_detections_buffer
            if (current_time - ts) <= self.aruco_detection_window_sec
        ]

        # Add new detections from the current message to the buffer
        for detected_id in msg.data:
            self.aruco_detections_buffer.append((current_time, detected_id))
            self.get_logger().debug(
                f"Aruco ID {detected_id} detected. Buffer size: {len(self.aruco_detections_buffer)}"
            )

        # Count occurrences of each unique tag ID in the current detection window
        tag_ids_in_window = [tag_id for ts, tag_id in self.aruco_detections_buffer]
        tag_counts = Counter(tag_ids_in_window)

        # Check if any tag has been detected the minimum required number of times
        for tag_id, count in tag_counts.items():
            if count >= self.aruco_min_detections:
                self.get_logger().info(
                    f"Aruco ID {tag_id} detected {count} times within {self.aruco_detection_window_sec} seconds! Stopping navigation."
                )
                self.mission_state = (
                    MissionState.FOUND_ARUCO_MARKER
                )  # Update mission state
                self.aruco_stop_triggered_id = (
                    tag_id  # Store the ID that caused the stop
                )
                self.navigator.cancelTask()  # Cancel the current navigation goal
                self.reset()  # Reset state variables, which will clear the Aruco buffer
                # Publish light code to indicate completion/stop
                light_msg = Int8()
                light_msg.data = self.nav_completed_light_code
                self.lights_publisher.publish(light_msg)
                self.get_logger().info(
                    f"Found Aruco Tag: {tag_id}"
                )  # Confirm which tag was found
                return  # Stop processing if a tag has triggered the stop

    def geopose_server(
        self, request: NavToGPSGeopose.Request, response: NavToGPSGeopose.Response
    ) -> NavToGPSGeopose.Response:
        """
        Service server to receive a new GPS goal and initiate navigation.
        Resets previous state and sets the mission to NAV_TO_GOAL.
        """
        self.get_logger().info(f"Received a new GPS goal: {request.goal}")
        self.final_lat_lon = (
            request.goal.position.latitude,
            request.goal.position.longitude,
            request.goal.position.altitude,
            request.goal.orientation,
        )
        self.reset()  # Reset any previous state, including Aruco buffer and triggered ID
        self.mission_state = (
            MissionState.NAV_TO_GOAL
        )  # Set state to navigate to the final goal
        response.success = True  # Acknowledge receipt of the goal

        # Reset the timer to trigger immediately and process the new goal
        self.timer.reset()

        # Publish light code to indicate navigation is active
        msg = Int8()
        msg.data = self.nav_activate_light_code
        self.lights_publisher.publish(msg)

        return response

    def cancel_nav_server(
        self, request: Trigger.Request, response: Trigger.Response
    ) -> Trigger.Response:
        """
        Service server to cancel the current navigation task.
        Sets the mission state to NAV_CANCELLED and publishes a cancellation light code.
        """
        self.get_logger().info("Received request to cancel navigation.")
        self.navigator.cancelTask()  # Cancel any active navigation
        self.reset()  # Reset state variables
        self.mission_state = MissionState.NAV_CANCELLED  # Set state to cancelled

        # Publish light code for cancellation
        msg = Int8()
        msg.data = self.nav_cancelled_light_code
        self.lights_publisher.publish(msg)

        self.get_logger().info("Navigation cancelled.")
        response.success = True  # Indicate successful cancellation for Trigger service
        response.message = (
            "Navigation cancelled by user."  # Provide a descriptive message
        )
        return response

    def convert_lat_lon_to_pose(self, latitude, longitude, altitude, orientation):
        """
        Converts a latitude, longitude, altitude, and orientation to a PoseStamped
        in the robot's map frame using the /fromLL service.
        """
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
                """Callback for the asynchronous service call."""
                nonlocal event
                event.set()

            # Call the service asynchronously
            future = self.localizer_client.call_async(req)
            # Add a done callback to signal when the future is complete
            future.add_done_callback(done_callback)
            # Wait for the event to be set, indicating the service call is done, with a timeout
            event.wait(timeout=5.0)

            if future.done():  # Check if the future completed
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
                    return target_pose
                else:
                    self.get_logger().error(
                        "Failed to convert lat/lon to map pose: Service returned no result."
                    )
                    return None
            else:
                self.get_logger().error(
                    "Failed to convert lat/lon to map pose: Service call timed out."
                )
                return None
        except Exception as e:
            self.get_logger().error(f"Error during lat/lon conversion: {e}")
            return None

    def euclidean_distance(self, pose1: PoseStamped, pose2: PoseStamped):
        """Calculates the Euclidean distance between two poses (ignoring z-axis)."""
        if pose1 is None or pose2 is None:
            return float("inf")
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def reset(self):
        """
        Resets the mission state and all related variables,
        preparing the node for a new navigation command.
        """
        self.goal_handle = None
        self.final_lat_lon = None
        self.aruco_detections_buffer = []  # Clear Aruco buffer on reset
        self.aruco_stop_triggered_id = None  # Clear the triggered Aruco ID
        self.mission_state = MissionState.DO_NOTHING  # Default to DO_NOTHING

    def timer_callback(self):
        """
        Main timer callback that handles all mission logic, state transitions,
        and monitors the navigation progress.
        """
        # Do nothing if the robot is in a non-active mission state
        if self.mission_state == MissionState.DO_NOTHING:
            return
        elif self.mission_state == MissionState.NAV_CANCELLED:
            return
        elif self.mission_state == MissionState.FOUND_ARUCO_MARKER:
            return

        # Logic for when the robot is actively navigating to a goal
        elif self.mission_state == MissionState.NAV_TO_GOAL:
            if self.final_lat_lon is None:
                self.get_logger().warn(
                    "Error in NAV_TO_GOAL: final_lat_lon is None. Resetting mission."
                )
                self.reset()
                return
            elif self.current_robot_pose is None:
                self.get_logger().warn(
                    "Error in NAV_TO_GOAL: current robot pose is not available."
                )
                return

            # Convert the final GPS goal to a PoseStamped in the map frame
            final_pose = self.convert_lat_lon_to_pose(*self.final_lat_lon)
            if final_pose is None:
                self.get_logger().error(
                    "Error in NAV_TO_GOAL: Failed to convert final lat/lon to map pose. Resetting mission."
                )
                self.reset()
                return

            # If no navigation goal is currently active, send the final goal to Nav2
            if self.goal_handle is None:
                self.get_logger().info(
                    f"Starting navigation to final goal: x={final_pose.pose.position.x:.2f}, y={final_pose.pose.position.y:.2f}"
                )
                self.goal_handle = self.navigator.goToPose(final_pose)
                self.get_logger().info(
                    f"Called goToPose for final pose. GoalHandle: {self.goal_handle}"
                )
                return  # Wait for the next timer cycle to check task status

            # Check if the navigation task is complete
            if self.navigator.isTaskComplete():
                self.get_logger().info("Final goal complete!")

                result = self.navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("Final goal succeeded!")
                    # This block is reached if navigation completed successfully without Aruco cancellation.
                    if self.aruco_stop_triggered_id is None:
                        self.get_logger().info(
                            "Navigated to goal but did not find the required Aruco tag within the specified window."
                        )
                    msg = Int8()
                    msg.data = self.nav_completed_light_code
                    self.lights_publisher.publish(msg)
                elif result == TaskResult.CANCELED:
                    # This block is reached if navigation was cancelled (e.g., by Aruco detection or user service call).
                    if self.aruco_stop_triggered_id is not None:
                        self.get_logger().info(
                            f"Found Aruco Tag {self.aruco_stop_triggered_id}, stopping navigation."
                        )
                    else:
                        self.get_logger().info(
                            "Final goal was canceled by user request or other reason!"
                        )
                    msg = Int8()
                    msg.data = self.nav_cancelled_light_code
                    self.lights_publisher.publish(msg)
                elif result == TaskResult.FAILED:
                    self.get_logger().error("Final goal failed!")
                    msg = Int8()
                    msg.data = self.nav_cancelled_light_code
                    self.lights_publisher.publish(msg)

                self.reset()  # Reset all state variables after task completion or failure


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
