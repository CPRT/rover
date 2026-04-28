import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor

import cv2
import numpy as np
from pathlib import Path
import threading
import queue
import sys

# Import OpenCV compatibility layer
from . import cv2_utils

try:
    from ultralytics import YOLO

    ULTRALYTICS_AVAILABLE = True
except ImportError:
    ULTRALYTICS_AVAILABLE = False


class YOLODetector(Node):
    def __init__(self):
        super().__init__("yolo_detector")

        if not ULTRALYTICS_AVAILABLE:
            self.get_logger().error(
                "Ultralytics YOLO not available! Install with: pip install ultralytics"
            )
            sys.exit(1)

        self.cv_bridge = CvBridge()

        # Thread-safe queue for images
        self.image_queue = queue.Queue(maxsize=0)

        # Camera intrinsic parameters
        self.fx = 0
        self.fy = 0
        self.cx = 0
        self.cy = 0

        # Declare ROS2 parameters
        self.declare_parameter(
            "model_path", "", ParameterDescriptor(description="Path to YOLO model file")
        )
        self.declare_parameter(
            "aruco_dict",
            "4X4_100",
            ParameterDescriptor(description="ArUco dictionary to use"),
        )
        self.declare_parameter(
            "aruco_marker_size",
            0.1,
            ParameterDescriptor(description="ArUco marker size in meters"),
        )
        self.declare_parameter(
            "confidence_threshold",
            0.5,
            ParameterDescriptor(description="Confidence threshold"),
        )
        self.declare_parameter(
            "video_output_path",
            "/tmp/yolo_detections.mp4",
            ParameterDescriptor(description="Path to save output video"),
        )
        self.declare_parameter(
            "video_fps", 10.0, ParameterDescriptor(description="Output video FPS")
        )
        self.declare_parameter(
            "use_compressed",
            False,
            ParameterDescriptor(description="Use compressed image topic"),
        )
        self.declare_parameter(
            "label_map", "", ParameterDescriptor(description="Label mapping")
        )

        # Get parameters
        model_path = self.get_parameter("model_path").value
        self.confidence_threshold = self.get_parameter("confidence_threshold").value
        self.video_output_path = self.get_parameter("video_output_path").value
        self.video_fps = self.get_parameter("video_fps").value
        self.use_compressed = self.get_parameter("use_compressed").value
        label_map_str = self.get_parameter("label_map").value
        aruco_dict_name = self.get_parameter("aruco_dict").value
        self.aruco_marker_size = self.get_parameter("aruco_marker_size").value

        if not model_path:
            self.get_logger().error("model_path parameter is required!")
            sys.exit(1)

        self.use_aruco = model_path.lower() == "aruco"

        # Initialization of Models (YOLO / ArUco)
        if self.use_aruco:
            self.get_logger().info(
                f"Using ArUco detection with dictionary: {aruco_dict_name}"
            )
            try:
                aruco_dict_attr = f"DICT_{aruco_dict_name.upper()}"
                if hasattr(cv2.aruco, aruco_dict_attr):
                    dict_id = getattr(cv2.aruco, aruco_dict_attr)
                    self.aruco_dict = cv2_utils.get_aruco_dictionary(dict_id)
                    self.aruco_params = cv2_utils.create_detector_parameters()
                else:
                    raise ValueError(f"Invalid ArUco dictionary: {aruco_dict_name}")
            except Exception as e:
                self.get_logger().error(f"Failed to initialize ArUco detector: {e}")
                sys.exit(1)
        else:
            self.get_logger().info(f"Loading YOLO model from: {model_path}")
            try:
                self.model = YOLO(model_path)
                self.class_names = (
                    self.model.names if hasattr(self.model, "names") else {}
                )

                # Apply label remapping
                self.label_remap = {}
                if label_map_str:
                    for mapping in label_map_str.split(","):
                        old_label, new_label = mapping.split(":")
                        try:
                            old_label = int(old_label.strip())
                        except ValueError:
                            old_label = old_label.strip()
                        self.label_remap[old_label] = new_label.strip()

                    for old_label, new_label in self.label_remap.items():
                        if old_label in self.class_names:
                            self.class_names[old_label] = new_label
            except Exception as e:
                self.get_logger().error(f"Failed to load YOLO model: {e}")
                sys.exit(1)

        # Video writer setup
        self.video_writer = None
        self.video_width = None
        self.video_height = None

        # Tracking variables for video stability and logging
        self.first_frame_time = None
        self.frames_written = 0
        self.processed_frames = 0
        self.duplicated_frames = 0
        self.dropped_frames = 0

        # Subscriptions
        if self.use_compressed:
            self.create_subscription(
                CompressedImage,
                "/zed/zed_node/left/image_rect_color/compressed",
                self.compressed_image_callback,
                10,
            )
        else:
            self.create_subscription(
                Image,
                "/zed/zed_node/left/image_rect_color",
                self.rgb_image_callback,
                10,
            )

        self.create_subscription(
            CameraInfo, "/zed/zed_node/left/camera_info", self.camera_info_callback, 10
        )

        # Publisher
        self.annotated_image_pub = self.create_publisher(
            Image, "/computer_vision/yolo_annotated_image", 10
        )

        # Start background processing thread
        self.is_running = True
        self.worker_thread = threading.Thread(target=self._processing_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("YOLO Detector Node Initialized with Queue Processing")

    def rgb_image_callback(self, msg: Image):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.image_queue.put((msg.header, cv_image))
        except Exception as e:
            self.get_logger().error(f"Error in RGB image callback: {e}")

    def compressed_image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                self.image_queue.put((msg.header, cv_image))
        except Exception as e:
            self.get_logger().error(f"Error in compressed image callback: {e}")

    def camera_info_callback(self, msg: CameraInfo):
        self.fx, self.cx = msg.k[0], msg.k[2]
        self.fy, self.cy = msg.k[4], msg.k[5]

    def _init_video_writer(self, height, width):
        try:
            output_path = Path(self.video_output_path)
            output_path.parent.mkdir(parents=True, exist_ok=True)
            fourcc = cv2.VideoWriter_fourcc(*"mp4v")
            self.video_writer = cv2.VideoWriter(
                str(output_path), fourcc, self.video_fps, (width, height)
            )

            if self.video_writer.isOpened():
                self.get_logger().info(
                    f"Video writer initialized: {width}x{height} @ {self.video_fps} fps"
                )
            else:
                self.video_writer = None
        except Exception as e:
            self.get_logger().error(f"Error initializing video writer: {e}")

    def _processing_loop(self):
        while self.is_running and rclpy.ok():
            try:
                # 0.5 sec timeout allows the loop to cleanly exit if Ctrl+C is pressed
                header, cv_image = self.image_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            if self.video_writer is None:
                self._init_video_writer(cv_image.shape[0], cv_image.shape[1])

            try:
                if self.use_aruco:
                    annotated_image = self._process_aruco(cv_image)
                else:
                    annotated_image = self._process_yolo(cv_image)

                self.processed_frames += 1

                if self.video_writer is not None:
                    frame_time_sec = header.stamp.sec + header.stamp.nanosec * 1e-9

                    if self.first_frame_time is None:
                        self.first_frame_time = frame_time_sec
                        self.video_writer.write(annotated_image)
                        self.frames_written = 1
                    else:
                        elapsed_time = frame_time_sec - self.first_frame_time
                        target_frame_count = int(elapsed_time * self.video_fps) + 1
                        frames_to_write = target_frame_count - self.frames_written

                        if frames_to_write > 0:
                            if frames_to_write > self.video_fps * 2:
                                self.get_logger().warn(
                                    f"Large time gap detected. Padding {frames_to_write} frames."
                                )

                            # Calculate duplicates (if we write 2 frames, 1 is a duplicate)
                            if frames_to_write > 1:
                                self.duplicated_frames += frames_to_write - 1

                            for _ in range(frames_to_write):
                                self.video_writer.write(annotated_image)
                                self.frames_written += 1
                        else:
                            # If frames_to_write is 0 or less, the frame is skipped from the video
                            self.dropped_frames += 1

                # Publish to ROS
                annotated_msg = self.cv_bridge.cv2_to_imgmsg(
                    annotated_image, encoding="bgr8"
                )
                annotated_msg.header = header
                self.annotated_image_pub.publish(annotated_msg)

                # Info logging once every second (roughly matching video_fps)
                if self.processed_frames % int(self.video_fps) == 0:
                    self.get_logger().info(
                        f"Stats | Queue: {self.image_queue.qsize()} | "
                        f"Processed: {self.processed_frames} | "
                        f"Duplicated: {self.duplicated_frames} | "
                        f"Dropped: {self.dropped_frames}"
                    )

            except Exception as e:
                self.get_logger().error(f"Error processing image: {e}")
            finally:
                self.image_queue.task_done()

    def _process_yolo(self, image):
        results = self.model(image, conf=self.confidence_threshold, verbose=False)
        return results[0].plot()

    def _process_aruco(self, image):
        annotated_image = image.copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2_utils.detect_markers(
            gray, self.aruco_dict, self.aruco_params
        )

        if ids is not None and len(ids) > 0:
            cv2_utils.draw_detected_markers(annotated_image, corners, ids)
            if self.fx > 0 and self.fy > 0:
                camera_matrix = np.array(
                    [[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]],
                    dtype=np.float32,
                )
                dist_coeffs = np.zeros((4, 1))

                for i in range(len(ids)):
                    rvecs, tvecs, _ = cv2_utils.estimate_pose_single_markers(
                        corners[i], self.aruco_marker_size, camera_matrix, dist_coeffs
                    )
                    cv2_utils.draw_axis(
                        annotated_image,
                        camera_matrix,
                        dist_coeffs,
                        rvecs[0],
                        tvecs[0],
                        self.aruco_marker_size * 0.5,
                    )

        return annotated_image

    def destroy_node(self):
        self.get_logger().info("Shutting down... finishing up current processes.")
        self.is_running = False

        # Give the worker thread time to exit smoothly
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=3.0)

        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info("--------------------------------------------------")
            self.get_logger().info(
                f"VIDEO SAVED SUCCESSFULLY TO: {self.video_output_path}"
            )
            self.get_logger().info(
                f"FINAL STATS: {self.processed_frames} Processed | {self.frames_written} Written to Video"
            )
            self.get_logger().info("--------------------------------------------------")

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YOLODetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Catch the Ctrl+C directly to prevent ugly stack traces
        if node:
            node.get_logger().info("Ctrl+C pressed. Safely exiting...")
    except Exception as e:
        print(f"Node Error: {e}")
    finally:
        # **CRITICAL FIX**: Explicitly call destroy_node to release the video file
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
