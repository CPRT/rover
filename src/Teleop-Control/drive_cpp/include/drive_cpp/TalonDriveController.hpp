#ifndef TALON_DRIVE_CONTROLLER_HPP
#define TALON_DRIVE_CONTROLLER_HPP

#include "WheelControl.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace geometry_msgs::msg;
using namespace nav_msgs::msg;

/**
 * @brief The TalonDriveController class is responsible for controlling a
 * robot's drive system using Talon motors. It subscribes to a twist message to
 * control the robot's motion and publishes odometry information. Assumes
 * diff drive and equal number of left and right wheels
 */
class TalonDriveController : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new TalonDriveController object.
   */
  explicit TalonDriveController(const rclcpp::NodeOptions& options);

 private:
  /**
   * @brief Callback to publish odometry information.
   */
  void odom_pub_callback();

  /**
   * @brief Callback to control the motor velocities based on the twist command.
   */
  void control_timer_callback();

  /**
   * @brief Callback to process incoming Twist messages.
   *
   * @param msg The Twist message containing linear and angular velocity
   * commands.
   */
  void twist_callback(const Twist::SharedPtr msg);

  /**
   * @brief The maximum speed the robot can travel at, in meters per second.
   */
  double maxSpeed_;

  /**
   * @brief The width of the robot's base, used for computing turn radius and
   * angular velocity.
   */
  double baseWidth_;

  /**
   * @brief Flag indicating whether to publish odometry information.
   */
  bool pubOdom_;

  /**
   * @brief Frame id used for odometry output
   */
  std::string frameId_;

  /**
   * @brief Covariance value for angular odometry
   */
  double angularCov_;

  /**
   * @brief Covariance value for linear odometry
   */
  double linearCov_;

  /**
   * @brief Circumference of the wheels, used for computing velocity.
   */
  double wheelCircumference_;

  /**
   * @brief Ratio of expected slip on angular movement (must be > 0 and <=1)
   * ~0 means can not turn, 1 means no slip
   */
  double angularSlipRatio_;

  /**
   * @brief Flag indicating whether to publish electrical status
   * information. (Not implemented)
   */
  bool pubElec_;

  /**
   * @brief Timeout before shutting down motors if no control messages are
   * received.
   */
  double timeout_;

  /**
   * @brief The timestamp of the last control message.
   */
  double lastTimestamp_;

  /**
   * @brief The list of wheels controlled by this drive controller.
   */
  std::vector<WheelControl> wheels_;

  bool low_latency_mode_;

  /**
   * @brief The list of motor status subscriptions, one for each wheel.
   */
  std::vector<rclcpp::Subscription<MotorStatus>::SharedPtr> statusSubs_;

  /**
   * @brief The subscription to the Twist messages for controlling the robot's
   * motion.
   */
  rclcpp::Subscription<Twist>::SharedPtr twistSub_;

  /**
   * @brief The publisher for sending odometry information.
   */
  rclcpp::Publisher<Odometry>::SharedPtr odomPub_;

  /**
   * @brief Timer to periodically send control commands to the wheels.
   */
  rclcpp::TimerBase::SharedPtr timer_;

  /**
   * @brief Timer to periodically publish odometry data.
   */
  rclcpp::TimerBase::SharedPtr odomTimer_;
};

#endif  // TALON_DRIVE_CONTROLLER_HPP
