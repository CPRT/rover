#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/srv/move_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

class arm_manual : public rclcpp::Node {
public:
  arm_manual();
  void processJoystickInput(const sensor_msgs::msg::Joy::SharedPtr joystickMsg);

private:
  interfaces::srv::MoveServo::Response sendRequest(int port, int pos);
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  void handleArm(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void setServoPosition(int port, int position);
  double getThrottleValue(const sensor_msgs::msg::Joy::SharedPtr joystickMsg);

  void servoRequest(int req_port, int req_pos);
  void declareParameters();
  void loadParameters();

  // Parameters
  int8_t kBaseAxis;         ///< Axis for movement of the base
  int8_t kWristRoll;        ///< Axis for the roll of the wrist joint
  int8_t kWristYawPositive; ///< Button for the wrist join to rotate in the
                            ///< positive direction
  int8_t kWristYawNegative; ///< Button for the wrist join to rotate in the
                            ///< negative direction
  int8_t kAct1Axis;         ///< Axis for the upper linear actuator control
  int8_t kAct2Axis;         ///< Axis for the lower linear actuator control
  int8_t kElbowYaw;         ///< Axis for the yaw of the elbow joint

  int8_t kClawOpen;  ///< Button for claw to open.
  int8_t kClawClose; ///< Button for claw to close.

  int8_t kSimpleForward;  ///< Button to move the end effector forward
  int8_t kSimpleBackward; ///< Button to move the end effector backward

  int8_t kThrottleAxis; ///< Axis for throttle control.

  double kThrottleMax; ///< Maximum throttle value from joystick.
  double kThrottleMin; ///< Minimum throttle value from joystick.

  // Publishers
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr
      joint_pub_; ///< Publisher for joint jog messages.

  // Message Messages
  control_msgs::msg::JointJog joint_msg_;

  // Servo Members
  rclcpp::Client<interfaces::srv::MoveServo>::SharedPtr servo_client_;

  // Servo Constants
  int kServoPort;
  int kServoMin;
  int kServoMax;
  int kClawMax;
  int kClawMin;
  double act1Scaler_;
  double act2Scaler_;
  int8_t servoPos_;
  bool buttonPressed_;

  double current_light_pwm_; ///< Current PWM value for lights.

  bool camera_service_available_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      twist_pub_; ///< Publisher for Twist messages.
};