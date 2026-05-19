#include "drive.hpp"

drive::drive() : Node("drive_node"), initialized_(false) {
  declare_parameters();
  load_parameters();
  twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  servo_y_pub_ = this->create_publisher<std_msgs::msg::Float32>("/tilt", 10);
  servo_x_pub_ = this->create_publisher<std_msgs::msg::Float32>("/pan", 10);
  camera_client_ =
      this->create_client<interfaces::srv::VideoOut>("/start_video");
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&drive::drive_control, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Drive controller started");
  servo_y_ = kDefaultServoY;
  servo_x_ = kDefaultServoX;
  servo_mast_ = 0;
};
void drive::setCarousell() {
  interfaces::srv::VideoOut::Request drive;
  interfaces::srv::VideoOut::Request drive_eef;
  interfaces::srv::VideoOut::Request eef_drive;
  interfaces::srv::VideoOut::Request eef;
  interfaces::srv::VideoOut::Request mast;

  drive.num_sources = 1;
  drive.sources.resize(drive.num_sources);
  drive.sources[0].name = "Drive";
  drive.sources[0].height = 100;
  drive.sources[0].width = 100;
  drive.sources[0].origin_x = 0;
  drive.sources[0].origin_y = 0;

  drive_eef.num_sources = 2;
  drive_eef.sources.resize(drive_eef.num_sources);
  drive_eef.sources[0] = drive.sources[0];
  drive_eef.sources[1].name = "EndEffector";
  drive_eef.sources[1].height = 20;
  drive_eef.sources[1].width = 20;
  drive_eef.sources[1].origin_x = 80;
  drive_eef.sources[1].origin_y = 80;

  eef.num_sources = 1;
  eef.sources.resize(eef.num_sources);
  eef.sources[0].name = "EndEffector";
  eef.sources[0].height = 100;
  eef.sources[0].width = 100;
  eef.sources[0].origin_x = 0;
  eef.sources[0].origin_y = 0;

  eef_drive.num_sources = 2;
  eef_drive.sources.resize(eef_drive.num_sources);
  eef_drive.sources[0] = eef.sources[0];
  eef_drive.sources[1].name = "Drive";
  eef_drive.sources[1].height = 20;
  eef_drive.sources[1].width = 20;
  eef_drive.sources[1].origin_x = 80;
  eef_drive.sources[1].origin_y = 80;

  mast.num_sources = 1;
  mast.sources.resize(mast.num_sources);
  mast.sources[0].name = "Drive";
  mast.sources[0].height = 100;
  mast.sources[0].width = 100;
  mast.sources[0].origin_x = 0;
  mast.sources[0].origin_y = 0;

  video_carousell_.push_back(drive);
  video_carousell_.push_back(eef);
  video_carousell_.push_back(drive_eef);
  video_carousell_.push_back(eef_drive);
  video_carousell_.push_back(mast);
  video_carousell_idx_ = 0;
}

void drive::camera_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  const auto &left_but = joystickMsg->buttons[6];
  const auto &right_but = joystickMsg->buttons[7];
  if (!left_but && !right_but) {
    cam_debounce_ = false;
    return;
  }
  if (left_but) {
    if (video_carousell_idx_ == 0) {
      video_carousell_idx_ = video_carousell_.size();
    }
    --video_carousell_idx_;
  }
  if (right_but) {
    if (++video_carousell_idx_ == video_carousell_.size()) {
      video_carousell_idx_ = 0;
    }
  }
  auto request = std::make_shared<interfaces::srv::VideoOut::Request>(
      video_carousell_[video_carousell_idx_]);

  camera_client_->async_send_request(request);
}

void drive::drive_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  if (!initialized_) {
    if (std::abs(joystickMsg->axes[kForwardAxis]) < kJoyDeadzone &&
        std::abs(joystickMsg->axes[kStrafeAxis]) < kJoyDeadzone &&
        std::abs(joystickMsg->axes[kYawAxis]) < kJoyDeadzone) {
      initialized_ = true;
    }
    return;
  }
  auto twist = geometry_msgs::msg::Twist();
  twist.linear.x = joystickMsg->axes[kForwardAxis] * kMaxLinear;
  twist.linear.y = joystickMsg->axes[kStrafeAxis] * kMaxLinear;
  twist.angular.z = joystickMsg->axes[kYawAxis] * kMaxAngular;

  twist_pub_->publish(twist);

  if (joystickMsg->buttons[kServoHomeButton]) {
    servo_y_ = kDefaultServoY;
    servo_x_ = kDefaultServoX;
  }

  if (joystickMsg->axes[kServoYAxis] > kJoyDeadzone) {
    servo_y_ -= kServoIncrement;
  } else if (joystickMsg->axes[kServoYAxis] < -kJoyDeadzone) {
    servo_y_ += kServoIncrement;
  }
  if (joystickMsg->buttons[kMastLeftButton]) {
    servo_mast_ -= kServoIncrement;
  } else if (joystickMsg->buttons[kMastRightButton]) {
    servo_mast_ += kServoIncrement;
  }
  if (joystickMsg->axes[kServoXAxis] > kJoyDeadzone) {
    servo_x_ += kServoIncrement;
  } else if (joystickMsg->axes[kServoXAxis] < -kJoyDeadzone) {
    servo_x_ -= kServoIncrement;
  }
  auto servo_msg = std_msgs::msg::Float32();
  servo_y_ = std::clamp(servo_y_, kServoMin, kServoMax);
  servo_msg.data = servo_y_;
  servo_y_pub_->publish(servo_msg);

  servo_x_ = std::clamp(servo_x_, kServoMin, kServoMax);
  servo_msg.data = servo_x_;
  servo_x_pub_->publish(servo_msg);
};

void drive::declare_parameters() {
  this->declare_parameter("max_linear", 1.0);
  this->declare_parameter("max_angular", 1.0);
  this->declare_parameter("forward_axis", 1);
  this->declare_parameter("yaw_axis", 2);
  this->declare_parameter("strafe_axis", 3);
  this->declare_parameter("servo_y_axis", 4);
  this->declare_parameter("servo_x_axis", 5);
  this->declare_parameter("servo_home_button", 8);
  this->declare_parameter("mast_left_button", 3);
  this->declare_parameter("mast_right_button", 1);
  this->declare_parameter("servo_increment", 0.01);
  this->declare_parameter("servo_min", -3.14);
  this->declare_parameter("servo_max", 3.14);
  this->declare_parameter("joy_deadzone", 0.01);
  this->declare_parameter("default_servo_x", 0.0);
  this->declare_parameter("default_servo_y", 0.0);
}
void drive::load_parameters() {
  this->get_parameter("max_linear", kMaxLinear);
  this->get_parameter("max_angular", kMaxAngular);
  this->get_parameter("forward_axis", kForwardAxis);
  this->get_parameter("yaw_axis", kYawAxis);
  this->get_parameter("strafe_axis", kStrafeAxis);
  this->get_parameter("servo_y_axis", kServoYAxis);
  this->get_parameter("servo_x_axis", kServoXAxis);
  this->get_parameter("servo_home_button", kServoHomeButton);
  this->get_parameter("mast_left_button", kMastLeftButton);
  this->get_parameter("mast_right_button", kMastRightButton);
  this->get_parameter("servo_increment", kServoIncrement);
  this->get_parameter("servo_min", kServoMin);
  this->get_parameter("servo_max", kServoMax);
  this->get_parameter("joy_deadzone", kJoyDeadzone);
  this->get_parameter("default_servo_x", kDefaultServoX);
  this->get_parameter("default_servo_y", kDefaultServoY);

  RCLCPP_INFO(this->get_logger(), "Loaded Max Linear: %f", kMaxLinear);
  RCLCPP_INFO(this->get_logger(), "Loaded Max Angular: %f", kMaxAngular);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drive>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}