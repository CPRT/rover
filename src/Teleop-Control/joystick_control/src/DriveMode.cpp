#include "DriveMode.hpp"

#include <algorithm>

DriveMode::DriveMode(rclcpp::Node* node)
    : Mode("Drive", node), camera_service_available_(false) {
  RCLCPP_INFO(node_->get_logger(), "Drive Mode");
  loadParameters();
  twist_pub_ =
      node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  servo_client_ =
      node_->create_client<interfaces::srv::MoveServo>("servo_service");
  pwm_pub_ =
      node_->create_publisher<std_msgs::msg::Float32>("servo_pwm_control", 10);

  // Wait for the service to be available
  if (servo_client_->wait_for_service(std::chrono::seconds(1))) {
    camera_service_available_ = true;
  } else {
    RCLCPP_WARN(node_->get_logger(), "Service not available after waiting");
  }
}

void DriveMode::processJoystickInput(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  handleTwist(joystickMsg);
  handleCam(joystickMsg);
  handleVideo(joystickMsg);
  handlePWM(joystickMsg);
}

double DriveMode::getThrottleValue(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  if (kThrottleAxis != -1) {
    double throttle = joystickMsg->axes[kThrottleAxis];
    throttle = std::max(kThrottleMin, std::min(kThrottleMax, throttle));
    // Normalize the throttle value to be between 0 and 1
    return (throttle - kThrottleMin) / (kThrottleMax - kThrottleMin);
  }
  return 1.0;
}

void DriveMode::handleTwist(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
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

void DriveMode::handleCam(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
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
  double current_time = node_->now().seconds();
  if (current_time - timestamp > 0.2) {
    pan_pos = std::max(0.0, std::min(360.0, pan_pos));
    tilt_pos = std::max(0.0, std::min(360.0, tilt_pos));
    timestamp = current_time;
    setServoPosition(kCamTiltPort, tilt_pos);
    setServoPosition(kCamPanPort, pan_pos);
  }
}

void DriveMode::handlePWM(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
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

void DriveMode::setServoPosition(int port, int position) const {
  if (camera_service_available_) {
    auto request = std::make_shared<interfaces::srv::MoveServo::Request>();
    request->port = port;
    request->pos = position;

    servo_client_->async_send_request(request);
  }
}

void DriveMode::handleVideo(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const {
  // TODO: Implement stream control
}

void DriveMode::declareParameters(rclcpp::Node* node) {
  node->declare_parameter("drive_mode.forward_axis", 1);
  node->declare_parameter("drive_mode.yaw_axis", 2);
  node->declare_parameter("drive_mode.cam_tilt_axis", 3);
  node->declare_parameter("drive_mode.cam_pan_axis", 4);
  node->declare_parameter("drive_mode.cam_reset", 5);
  node->declare_parameter("drive_mode.lights_up", 6);
  node->declare_parameter("drive_mode.lights_down", 7);
  node->declare_parameter("drive_mode.cruise_control", 8);
  node->declare_parameter("drive_mode.max_linear", 2.0);
  node->declare_parameter("drive_mode.max_angular", 2.0);
  node->declare_parameter("drive_mode.max_increment", 0.1);
  node->declare_parameter("drive_mode.min_speed", 0.1);
  node->declare_parameter("drive_mode.throttle.axis", 0);
  node->declare_parameter("drive_mode.throttle.max", 1.0);
  node->declare_parameter("drive_mode.throttle.min", -1.0);
  node->declare_parameter("drive_mode.cam_tilt_port", 0);
  node->declare_parameter("drive_mode.cam_pan_port", 1);
  node->declare_parameter("drive_mode.default_pan", 90.0);
  node->declare_parameter("drive_mode.default_tilt", 90.0);
  node->declare_parameter("drive_mode.camera_speed", 1.0);
}

void DriveMode::loadParameters() {
  node_->get_parameter("drive_mode.forward_axis", kForwardAxis);
  node_->get_parameter("drive_mode.yaw_axis", kYawAxis);
  node_->get_parameter("drive_mode.cam_tilt_axis", kCamTiltAxis);
  node_->get_parameter("drive_mode.cam_pan_axis", kCamPanAxis);
  node_->get_parameter("drive_mode.cam_reset", kCamReset);
  node_->get_parameter("drive_mode.lights_up", kLightsUp);
  node_->get_parameter("drive_mode.lights_down", kLightsDown);
  node_->get_parameter("drive_mode.cruise_control", kCruiseControl);
  node_->get_parameter("drive_mode.max_linear", kMaxLinear);
  node_->get_parameter("drive_mode.max_angular", kMaxAngular);
  node_->get_parameter("drive_mode.max_increment", kMaxIncrement);
  node_->get_parameter("drive_mode.min_speed", kMinSpeed);
  node_->get_parameter("drive_mode.throttle.axis", kThrottleAxis);
  node_->get_parameter("drive_mode.throttle.max", kThrottleMax);
  node_->get_parameter("drive_mode.throttle.min", kThrottleMin);
  node_->get_parameter("drive_mode.cam_tilt_port", kCamTiltPort);
  node_->get_parameter("drive_mode.cam_pan_port", kCamPanPort);
  node_->get_parameter("drive_mode.default_pan", kDefaultCamPan);
  node_->get_parameter("drive_mode.default_tilt", kDefaultCamTilt);
  node_->get_parameter("drive_mode.camera_speed", kCameraSpeed);
}
