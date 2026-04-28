import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor

import cv2
import numpy as np
from pathlib import Path
import threading
import queue
import sys


class ImageRecorder(Node):
    def __init__(self):
        super().__init__("image_recorder")

        self.cv_bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=0)

        # Declare parameters
        self.declare_parameter(
            "output_dir",
            "/tmp/recorded_images",
            ParameterDescriptor(description="Directory to save images"),
        )
        self.declare_parameter(
            "use_compressed",
            False,
            ParameterDescriptor(description="Use compressed image topic"),
        )
        self.declare_parameter(
            "image_format",
            ".jpg",
            ParameterDescriptor(description="Image format (.jpg or .png)"),
        )

        # Get parameters
        self.output_dir = Path(self.get_parameter("output_dir").value)
        self.use_compressed = self.get_parameter("use_compressed").value
        self.image_format = self.get_parameter("image_format").value

        # Create output directory
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f"Saving images to: {self.output_dir}")

        self.frames_saved = 0

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

        # Start background saving thread
        self.is_running = True
        self.worker_thread = threading.Thread(target=self._saving_loop, daemon=True)
        self.worker_thread.start()

        self.get_logger().info("Image Recorder Node Initialized")

    def rgb_image_callback(self, msg: Image):
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.image_queue.put((msg.header.stamp, cv_image))
        except Exception as e:
            self.get_logger().error(f"Error in RGB callback: {e}")

    def compressed_image_callback(self, msg: CompressedImage):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            if cv_image is not None:
                self.image_queue.put((msg.header.stamp, cv_image))
        except Exception as e:
            self.get_logger().error(f"Error in compressed callback: {e}")

    def _saving_loop(self):
        """Consumer: Pulls images from queue and saves them to disk"""
        while self.is_running and rclpy.ok():
            try:
                stamp, cv_image = self.image_queue.get(timeout=0.5)
            except queue.Empty:
                continue

            try:
                # Format filename using the exact ROS timestamp
                # Example: frame_1772152065_713301560.jpg
                filename = f"frame_{stamp.sec}_{stamp.nanosec:09d}{self.image_format}"
                filepath = self.output_dir / filename

                # Save the image
                cv2.imwrite(str(filepath), cv_image)
                self.frames_saved += 1

                if self.frames_saved % 30 == 0:
                    self.get_logger().info(
                        f"Saved {self.frames_saved} images. Queue size: {self.image_queue.qsize()}"
                    )

            except Exception as e:
                self.get_logger().error(f"Error saving image: {e}")
            finally:
                self.image_queue.task_done()

    def destroy_node(self):
        self.get_logger().info("Shutting down... saving remaining images in queue.")
        self.is_running = False

        # Give the worker thread a few seconds to flush the queue to disk
        if self.worker_thread.is_alive():
            self.worker_thread.join(timeout=5.0)

        self.get_logger().info("--------------------------------------------------")
        self.get_logger().info(
            f"SUCCESS: Saved {self.frames_saved} total images to {self.output_dir}"
        )
        self.get_logger().info("--------------------------------------------------")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImageRecorder()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Ctrl+C pressed. Safely exiting...")
    except Exception as e:
        print(f"Node Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
