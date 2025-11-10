import rclpy
from rclpy.node import Node
from geometry_msgs.msg import (
    PoseWithCovarianceStamped,
    Point,
    Quaternion,
    Pose,
    PoseWithCovariance,
)
from std_msgs.msg import Header
from typing import List


class StaticPoseRepublisher(Node):
    """
    A ROS 2 node that continuously publishes a fixed, static
    PoseWithCovarianceStamped message (set to the origin) at a high frequency.

    The timestamp is continuously updated, and the covariance matrix is preserved
    from the user-provided example for fidelity.
    """

    # The covariance array provided by the user's example message (36 elements)
    # This is a constant value representing the uncertainty, kept consistent
    # with the requested format.
    STATIC_COVARIANCE: List[float] = [
        4.411424185946089e-07,
        2.739288085251701e-08,
        -1.084223342218138e-07,
        1.0752466472752076e-08,
        -1.7559282916579377e-09,
        -6.794176776736549e-09,
        2.739290572151276e-08,
        3.1544121270599135e-07,
        -1.0688204099551513e-07,
        -4.124122199300473e-08,
        -3.153438399294828e-08,
        -3.7703589583770736e-08,
        -1.0842235553809587e-07,
        -1.068820125738057e-07,
        1.672603815450202e-07,
        7.836187698728736e-09,
        2.447248448333994e-08,
        1.3136301824090424e-08,
        1.075245315007578e-08,
        -4.124121488757737e-08,
        7.836191251442415e-09,
        3.513175883540498e-08,
        1.2356305312266613e-08,
        9.377173704194774e-09,
        -1.7559342868622707e-09,
        -3.153436978209356e-08,
        2.4472477377912583e-08,
        1.2356305312266613e-08,
        2.7540579594642622e-08,
        6.0083276132161245e-09,
        -6.794183882163907e-09,
        -3.7703564714774984e-08,
        1.3136300047733585e-08,
        9.377173704194774e-09,
        6.008327169126915e-09,
        1.6435310001838843e-08,
    ]

    def __init__(
        self,
        frequency: float = 10.0,
        topic_name: str = "/zed/zed_node/pose_with_covariance",
    ):
        super().__init__("static_pose_republisher")

        self.get_logger().info(
            f"StaticPoseRepublisher starting at {frequency} Hz on topic '{topic_name}'."
        )

        # Set the frequency for publishing (Hz)
        self.frequency = frequency

        # Publisher to the target topic
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, topic_name, 10
        )

        # Create the static message template once
        self.static_pose_msg = self._create_origin_pose_message()

        # Timer to publish the static message
        self.timer = self.create_timer(1.0 / self.frequency, self.publish_static_pose)

    def _create_origin_pose_message(self) -> PoseWithCovarianceStamped:
        """
        Creates a PoseWithCovarianceStamped message set to the origin (0,0,0)
        with identity orientation, using the specified frame_id and covariance.
        """
        msg = PoseWithCovarianceStamped()

        # 1. Header Frame ID: Preserve 'map' from the user's example
        msg.header.frame_id = "map"

        # 2. Pose: Set to the Origin (0,0,0, identity quaternion)
        origin_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
        )

        # 3. PoseWithCovariance: Combine pose and covariance
        msg.pose = PoseWithCovariance(
            pose=origin_pose,
            covariance=self.STATIC_COVARIANCE,
        )

        return msg

    def publish_static_pose(self):
        """
        Timer callback: updates the timestamp and publishes the static pose message.
        """
        # Update the timestamp to the current ROS time
        self.static_pose_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the message
        self.pose_publisher.publish(self.static_pose_msg)


def main(args=None):
    rclpy.init(args=args)

    # Configuration
    frequency_hz = 10.0  # Publishing frequency (Hz)
    target_topic = "/static_zed_pose"

    try:
        # Create the node and start spinning
        node = StaticPoseRepublisher(frequency=frequency_hz, topic_name=target_topic)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Shutdown ROS 2 when exiting
        if "node" in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
