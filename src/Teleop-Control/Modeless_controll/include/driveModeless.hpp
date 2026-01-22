
#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/srv/move_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class drive : public rclcpp::Node {
public:
  drive();
  void processJoystickInput_drive(
      const sensor_msgs::msg::Joy::SharedPtr joystickMsg);
  void
  processJoystickInput_arm(const sensor_msgs::msg::Joy::SharedPtr joystickMsg);

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

  void handleTwist(const sensor_msgs::msg::Joy::SharedPtr joystickMsg);
  void handleCam(const sensor_msgs::msg::Joy::SharedPtr joystickMsg);
  void handlePWM(const sensor_msgs::msg::Joy::SharedPtr joystickMsg);
  double getThrottleValue(const sensor_msgs::msg::Joy::SharedPtr joystickMsg);
  void setServoPosition(int port, int position);
  void handleVideo(const sensor_msgs::msg::Joy::SharedPtr joystickMsg);
  double getThrottleValue(const sensor_msgs::msg::Joy::SharedPtr joystickMsg);

  void declareParameters();
  void loadParameters();
  // Parameters
  int8_t kJoystickID; /// ID for seperating joysticks

  int8_t kForwardAxis;   ///< Axis for forward movement.
  int8_t kYawAxis;       ///< Axis for yaw (rotation).
  int8_t kCamTiltAxis;   ///< Axis for camera tilt.
  int8_t kCamPanAxis;    ///< Axis for camera pan.
  int8_t kCamReset;      ///< Button for resetting the camera.
  int8_t kLightsUp;      ///< Button for switching to the next camera.
  int8_t kLightsDown;    ///< Button for switching to the previous camera.
  int8_t kCruiseControl; ///< Button for enabling cruise control.
  int8_t kThrottleAxis;  ///< Axis for throttle control.
  int8_t kCamTiltPort;   ///< Port for camera tilt servo.
  int8_t kCamPanPort;    ///< Port for camera pan servo.

  double kThrottleMax;           ///< Maximum throttle value from joystick.
  double kThrottleMin;           ///< Minimum throttle value from joystick.
  double kMaxLinear;             ///< Maximum linear velocity.
  double kMaxAngular;            ///< Maximum angular velocity.
  double kMaxIncrement;          ///< Maximum increment for speed changes.
  double kMinSpeed;              ///< Minimum speed value.
  double kDefaultCamPan = 90.0;  ///< Default camera pan position.
  double kDefaultCamTilt = 90.0; ///< Default camera tilt position.
  double kCameraSpeed = 1.0;     ///< Speed for camera movement.

  double current_light_pwm_; ///< Current PWM value for lights.

  bool camera_service_available_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      twist_pub_; ///< Publisher for Twist messages.
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr
      pwm_pub_; ///< Publisher for video messages.

  rclcpp::Client<interfaces::srv::MoveServo>::SharedPtr servo_client_;
};