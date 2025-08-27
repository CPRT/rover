#ifndef JOYSTICK_CONTROL__ARMMANUAL_MODE_HPP_
#define JOYSTICK_CONTROL__ARMMANUAL_MODE_HPP_

#include "Mode.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/srv/move_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

/**
 * @class ArmManualMode
 * @brief A class that handles the manual arm control using a joystick
 *
 * This class inherits from the Mode class and processes joystick input to
 * control the arm manually, meaning each join is controlled individually.
 */
class ArmManualMode : public Mode {
 public:
  /**
   * @brief Constructor to the ArmManualMode class.
   *
   * @param node A pointer to the rclcpp::Node instance.
   */
  ArmManualMode(rclcpp::Node* node);

  /**
   * @brief Processes the joystick input.
   *
   * @param joystickMsg A shared pointer to the incoming sensor_msgs::msg::Joy
   * message.
   */
  void processJoystickInput(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;

  static void declareParameters(rclcpp::Node* node);

  /**
   * @brief Sends a request to move a servo
   * @param port The servo port number
   * @param pos The target position
   * @param min The minimum position limit of the servo
   * @param max The maximum position limit of the servo
   * @return The service response
   */
  interfaces::srv::MoveServo::Response sendRequest(int port, int pos) const;

  /**
   * @brief Wrapper for servo control
   * @param req_port The servo port number
   * @param req_pos The target position
   * @param req_min The minimum position limit
   * @param req_max The maximum position limit
   */
  void servoRequest(int req_port, int req_pos) const;

 private:
  /**
   * @brief creates and publishes the MotorControl msg based on joystickinput.
   *
   * @param joystickMsg A shared pointer to the sensor_msgs::msg::Joy message.
   */
  void handleTwist(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

  /**
   * @brief Gets the throttle value from the joystick input.
   *
   * @param joystickMsg A shared pointer to the sensor_msgs::msg::Joy message.
   * @return The throttle value as a double between 0 and one inclusive.
   */
  double getThrottleValue(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;

  void loadParameters();

  // Parameters
  int8_t kBaseAxis;          ///< Axis for movement of the base
  int8_t kWristRoll;         ///< Axis for the roll of the wrist joint
  int8_t kWristYawPositive;  ///< Button for the wrist join to rotate in the
                             ///< positive direction
  int8_t kWristYawNegative;  ///< Button for the wrist join to rotate in the
                             ///< negative direction
  int8_t kAct1Axis;          ///< Axis for the upper linear actuator control
  int8_t kAct2Axis;          ///< Axis for the lower linear actuator control
  int8_t kElbowYaw;          ///< Axis for the yaw of the elbow joint

  int8_t kThrottleAxis;  ///< Axis for the throttle value
  double kThrottleMax;   ///< Maximum throttle value from joystick.
  double kThrottleMin;   ///< Minimum throttle value from joystick.

  int8_t kClawOpen;   ///< Button for claw to open.
  int8_t kClawClose;  ///< Button for claw to close.

  int8_t kSimpleForward;   ///< Button to move the end effector forward
  int8_t kSimpleBackward;  ///< Button to move the end effector backward

  // Publishers
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr
      joint_pub_;  ///< Publisher for joint jog messages.

  // Message Messages
  control_msgs::msg::JointJog joint_msg_;

  // Servo Members
  rclcpp::Client<interfaces::srv::MoveServo>::SharedPtr servo_client_;

  // Servo Constants
  int8_t kServoPort;
  int8_t kServoMin;
  int8_t kServoMax;
  int8_t kClawMax;
  int8_t kClawMin;
  double act1Scaler_;
  double act2Scaler_;
  int8_t servoPos_;
  bool buttonPressed_;
};

#endif  // JOYSTICK_CONTROL__ARMMANUAL_MODE_HPP_