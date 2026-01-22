#include "driveModeless.hpp"
#include "std_msgs/msg/bool.hpp"
#include <algorithm>

drive::drive() : rclcpp::Node("modeless_drive_control") {

  this->declareParameters();
  this->loadParameters();

  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/controller_a/joy", 10,
      std::bind(&drive::processJoystickInput_drive, this,
                std::placeholders::_1));

  twist_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  servo_client_ =
      this->create_client<interfaces::srv::MoveServo>("servo_service");
  pwm_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("servo_pwm_control", 10);

  // Wait for the service to be available
  if (servo_client_->wait_for_service(std::chrono::seconds(1))) {
    camera_service_available_ = true;
  } else {
    RCLCPP_WARN(this->get_logger(), "Service not available after waiting");
  }
}

void drive::setServoPosition(int port, int position) {
  if (camera_service_available_) {
    auto request = std::make_shared<interfaces::srv::MoveServo::Request>();
    request->port = port;
    request->pos = position;

    servo_client_->async_send_request(request);
  }
}

void drive::processJoystickInput_drive(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  handleTwist(joystickMsg);
  handleCam(joystickMsg);
  handleVideo(joystickMsg);
  handlePWM(joystickMsg);
}

double
drive::getThrottleValue(const sensor_msgs::msg::Joy::SharedPtr joystickMsg) {
  if (kThrottleAxis != -1) {
    double throttle = joystickMsg->axes[kThrottleAxis];
    throttle = std::max(kThrottleMin, std::min(kThrottleMax, throttle));
    // Normalize the throttle value to be between 0 and 1
    return (throttle - kThrottleMin) / (kThrottleMax - kThrottleMin);
  }
  return 1.0;
}

void drive::handleTwist(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  static double last_forward = 0;
  double forward, yaw, throttle;
  throttle = getThrottleValue(joystickMsg);
  if (joystickMsg->buttons[kCruiseControl] == 1) {
    forward = last_forward;
  } else {
    forward = joystickMsg->axes[kForwardAxis] * throttle * kMaxLinear;
  }
  yaw = joystickMsg->axes[kYawAxis] * kMaxAngular;
  if (std::abs(forward) < kMinSpeed) {
    forward = 0;
  }
  if (std::abs(yaw) < kMinSpeed) {
    yaw = 0;
  }
  if (forward >= last_forward) {
    forward = std::min(forward, last_forward + kMaxIncrement);
  } else {
    forward = std::max(forward, last_forward - kMaxIncrement);
  }
  last_forward = forward;
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = forward;
  twist.angular.z = yaw;
  twist_pub_->publish(twist);
}

void drive::handleCam(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  static double tilt_pos = kDefaultCamTilt;
  static double pan_pos = kDefaultCamPan;
  static double timestamp = 0;
  double tilt = joystickMsg->axes[kCamTiltAxis];
  double pan = joystickMsg->axes[kCamPanAxis];
  tilt_pos += tilt * kCameraSpeed;
  pan_pos += pan * kCameraSpeed;
  if (joystickMsg->buttons[kCamReset] == 1) {
    tilt_pos = kDefaultCamTilt;
    pan_pos = kDefaultCamPan;
    // return to only send once button is released
    return;
  }

  double current_time = this->now().seconds();
  if (current_time - timestamp > 0.2) {
    pan_pos = std::max(0.0, std::min(360.0, pan_pos));
    tilt_pos = std::max(0.0, std::min(360.0, tilt_pos));
    timestamp = current_time;
    setServoPosition(kCamTiltPort, tilt_pos);
    setServoPosition(kCamPanPort, pan_pos);
  }
}

void drive::handlePWM(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  bool publish_pwm = false;
  if (joystickMsg->buttons[kLightsUp] == 1) {
    current_light_pwm_ = std::min(current_light_pwm_ + 1, 100.0);
    publish_pwm = true;
  } else if (joystickMsg->buttons[kLightsDown] == 1) {
    current_light_pwm_ = std::max(current_light_pwm_ - 1, 0.0);
    publish_pwm = true;
  }
  if (publish_pwm) {
    auto pwm_msg = std_msgs::msg::Float32();
    pwm_msg.data = current_light_pwm_;
    pwm_pub_->publish(pwm_msg);
  }
}

void drive::handleVideo(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  // TODO: Implement stream control
}

void drive::declareParameters() {
  this->declare_parameter("drive_mode.forward_axis", 1);
  this->declare_parameter("drive_mode.yaw_axis", 2);
  this->declare_parameter("drive_mode.cam_tilt_axis", 3);
  this->declare_parameter("drive_mode.cam_pan_axis", 4);
  this->declare_parameter("drive_mode.cam_reset", 5);
  this->declare_parameter("drive_mode.lights_up", 6);
  this->declare_parameter("drive_mode.lights_down", 7);
  this->declare_parameter("drive_mode.cruise_control", 8);
  this->declare_parameter("drive_mode.max_linear", 2.0);
  this->declare_parameter("drive_mode.max_angular", 2.0);
  this->declare_parameter("drive_mode.max_increment", 0.1);
  this->declare_parameter("drive_mode.min_speed", 0.1);
  this->declare_parameter("drive_mode.throttle.axis", 0);
  this->declare_parameter("drive_mode.throttle.max", 1.0);
  this->declare_parameter("drive_mode.throttle.min", -1.0);
  this->declare_parameter("drive_mode.cam_tilt_port", 0);
  this->declare_parameter("drive_mode.cam_pan_port", 1);
  this->declare_parameter("drive_mode.default_pan", 90.0);
  this->declare_parameter("drive_mode.default_tilt", 90.0);
  this->declare_parameter("drive_mode.camera_speed", 1.0);
}

void drive::loadParameters() {
  this->get_parameter("drive_mode.forward_axis", kForwardAxis);
  this->get_parameter("drive_mode.yaw_axis", kYawAxis);
  this->get_parameter("drive_mode.cam_tilt_axis", kCamTiltAxis);
  this->get_parameter("drive_mode.cam_pan_axis", kCamPanAxis);
  this->get_parameter("drive_mode.cam_reset", kCamReset);
  this->get_parameter("drive_mode.lights_up", kLightsUp);
  this->get_parameter("drive_mode.lights_down", kLightsDown);
  this->get_parameter("drive_mode.cruise_control", kCruiseControl);
  this->get_parameter("drive_mode.max_linear", kMaxLinear);
  this->get_parameter("drive_mode.max_angular", kMaxAngular);
  this->get_parameter("drive_mode.max_increment", kMaxIncrement);
  this->get_parameter("drive_mode.min_speed", kMinSpeed);
  this->get_parameter("drive_mode.throttle.axis", kThrottleAxis);
  this->get_parameter("drive_mode.throttle.max", kThrottleMax);
  this->get_parameter("drive_mode.throttle.min", kThrottleMin);
  this->get_parameter("drive_mode.cam_tilt_port", kCamTiltPort);
  this->get_parameter("drive_mode.cam_pan_port", kCamPanPort);
  this->get_parameter("drive_mode.default_pan", kDefaultCamPan);
  this->get_parameter("drive_mode.default_tilt", kDefaultCamTilt);
  this->get_parameter("drive_mode.camera_speed", kCameraSpeed);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  // Create the node (drive now inherits from rclcpp::Node)
  auto node = std::make_shared<drive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}