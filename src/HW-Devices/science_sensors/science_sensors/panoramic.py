import rclpy
import numpy as np
import cv2
import time
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from interfaces.srv import MoveServo
from interfaces.srv import VideoCapture


class PanoramicNode(Node):
    def __init__(self):
        super().__init__("panoramic_node")
        self.get_logger().info("Panoramic Node has been started.")
        self.load_params()
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.video_callback_group = MutuallyExclusiveCallbackGroup()
        self.servo_cli = self.create_client(
            MoveServo,
            "/servo_service",
        )
        self.video_cli = self.create_client(
            VideoCapture, "/capture_frame", callback_group=self.video_callback_group
        )
        self.pan_srv = self.create_service(
            VideoCapture,
            "/capture_panoramic",
            self.start,
            callback_group=self.callback_group,
        )

    def load_params(self):
        self.declare_parameter("tilt_port", 0)
        self.declare_parameter("pan_port", 1)
        self.declare_parameter("camera_name", "Drive")
        self.declare_parameter("images", 10)
        self.declare_parameter("sleep", 2.0)
        self.pan_port = (
            self.get_parameter("pan_port").get_parameter_value().integer_value
        )
        self.tilt_port = (
            self.get_parameter("tilt_port").get_parameter_value().integer_value
        )
        self.camera_name = (
            self.get_parameter("camera_name").get_parameter_value().string_value
        )
        self.num_images = (
            self.get_parameter("images").get_parameter_value().integer_value
        )
        self.sleep_time = self.get_parameter("sleep").get_parameter_value().double_value
        self.get_logger().info(
            f"Parameters - pan_port: {self.pan_port}, tilt_port: {self.tilt_port}, "
            f"camera_name: {self.camera_name}, images: {self.num_images}, sleep: {self.sleep_time}"
        )

    def move_servo(self, port, angle):
        if not self.servo_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Servo service not available, exiting...")
            return
        request = MoveServo.Request()
        request.port = port
        request.pos = angle
        self.servo_cli.call_async(request)

    def move_panoramic(self, pan_angle, tilt_angle):
        self.move_servo(self.pan_port, pan_angle)
        self.move_servo(self.tilt_port, tilt_angle)

    def capture_image(self) -> np.ndarray:
        if not self.video_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Video capture service not available, exiting...")
            return
        request = VideoCapture.Request()
        request.source = self.camera_name
        future = self.video_cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result is None:
            self.get_logger().error("Failed to capture image")
            return None

        image = np.frombuffer(result.image.data, dtype=np.uint8)
        if result.image.format == "png":
            image = cv2.imdecode(image, cv2.IMREAD_UNCHANGED)
        elif result.image.format == "jpeg":
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        else:
            self.get_logger().warn("Format not set. Assuming jpeg")
            image = cv2.imdecode(image, cv2.IMREAD_COLOR)

        if image is None or image.size == 0:
            self.get_logger().error("Failed to decode image")
            return None
        return image

    def construct_panoramic(self, images):
        # Assuming images is a list of numpy arrays
        if len(images) == 0:
            self.get_logger().warn("No images found")
            return None
        stitcher = cv2.Stitcher_create()
        self.get_logger().info("Stitching started.")
        status, panoramic_image = stitcher.stitch(images)
        self.get_logger().info("Stitching finished.")
        if status == cv2.Stitcher_OK:
            self.get_logger().info("Stitching successful.")
        else:
            self.get_logger().error(f"Stitching failed. Error code: {status}")
            return None
        return panoramic_image

    def start(self, request, response):
        self.get_logger().info("Panoramic capture started")
        images = []
        tilt_angle = 45
        for i in range(self.num_images):
            pan_angle = int(i * (360 / self.num_images))
            self.move_panoramic(pan_angle, tilt_angle)
            time.sleep(self.sleep_time)
            image = self.capture_image()
            if image is None:
                self.get_logger().error("Failed to capture image")
                response.success = False
                return response
            images.append(image)
            self.get_logger().info(f"Captured image {i + 1}/{self.num_images}")
        image = self.construct_panoramic(images)
        if image is None:
            self.get_logger().error("Failed to construct panoramic image")
            response.success = False
            return response
        # Encode image to send back
        _, buffer = cv2.imencode(".jpeg", image)
        response.image.data = buffer.tobytes()
        response.image.format = "jpeg"
        response.success = True
        if request.filename != "":
            try:
                with open(request.filename, "wb") as f:
                    f.write(buffer)
                self.get_logger().info(f"Panoramic image saved to {request.filename}")
            except Exception as e:
                self.get_logger().error(
                    f"Failed to save image to {request.filename}: {str(e)}"
                )
        self.get_logger().info("Panoramic capture completed")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PanoramicNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Panoramic Node is shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
