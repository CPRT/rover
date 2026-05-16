import rclpy
import numpy as np
import numpy.typing as npt
import cv2
import time
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from interfaces.srv import VideoCapture, VideoOut
from interfaces.msg import VideoSource
from threading import Event
from std_msgs.msg import Float32


class PanoramicNode(Node):
    def __init__(self):
        super().__init__("panoramic_node")
        self.get_logger().info("Panoramic Node has been started.")
        self.load_params()
        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.video_callback_group = MutuallyExclusiveCallbackGroup()
        self.servo_pan_pub = self.create_publisher(Float32, "/mast_angle", 10)
        self.video_cli = self.create_client(
            VideoCapture, "/capture_frame", callback_group=self.video_callback_group
        )
        self.set_cam_cli = self.create_client(VideoOut, "/start_video", callback_group=self.video_callback_group)
        self.pan_srv = self.create_service(
            VideoCapture,
            "/capture_panoramic",
            self.start,
            callback_group=self.callback_group,
        )

    def load_params(self):
        self.declare_parameter("images", 10)
        self.declare_parameter("sleep", 2.0)
        self.num_images = (
            self.get_parameter("images").get_parameter_value().integer_value
        )
        self.sleep_time = self.get_parameter("sleep").get_parameter_value().double_value

    @staticmethod
    def move_servo(publisher, angle):
        msg = Float32()
        msg.data = angle
        publisher.publish(msg)

    def capture_image(self) -> npt.NDArray[np.uint8] | None:
        if not self.video_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Video capture service not available, exiting...")
            return None
        request = VideoCapture.Request()
        # Use an Event to wait for the async call to complete
        # Using ros2 executor blocking calls creates deadlock after first call
        event = Event()

        def done_callback(future):
            nonlocal event
            event.set()

        future = self.video_cli.call_async(request)
        future.add_done_callback(done_callback)
        event.wait(10)  # wait for max 10 seconds
        if not future.done():
            self.get_logger().error("Video capture service call timed out")
            return None

        result = future.result()

        if result is None or not result.success or len(result.image.data) == 0:
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
        if not self.set_cam_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Could not set camera, exiting...")
            response.success = False
            return response
        video_start_req = VideoOut.Request()
        source = VideoSource()
        source.name = "Mast"
        source.width = 100
        source.height = 100
        video_start_req.sources = [source]
        
        # Use an Event to wait for the async call to complete
        # Using ros2 executor blocking calls creates deadlock after first call
        event = Event()

        def done_callback(future):
            nonlocal event
            event.set()

        future = self.set_cam_cli.call_async(video_start_req)
        future.add_done_callback(done_callback)
        event.wait(10)  # wait for max 10 seconds
        if not future.done():
            self.get_logger().error("Could not set camera")
            response.success = False
            return response

        result = future.result()
        images = []
        for i in range(self.num_images):
            pan_angle = i * (6.28 / self.num_images)
            self.get_logger().info(f"Moving servo to {pan_angle} radians")
            self.move_servo(self.servo_pan_pub, pan_angle)
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
