#include "science.hpp"

science::science() : Node("science_node") {
  declare_parameters();
  load_parameters();
  drill_pub_ =
      this->create_publisher<std_msgs::msg::Float32>("ros_phoenix/drill", 10);
  elevator_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      "ros_phoenix/elevator", 10);
  test_servo_pub_ = this->create_publisher<std_msgs::msg::UInt32>(
      "/micro_ros_subscriber", 10);
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&science::science_control, this, std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "Science controller started");
};

void science::science_control(
    std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
  auto drill_msg_ = std_msgs::msg::Float32();
  auto elevator_msg_ = std_msgs::msg::Float32();
  if (joystickMsg->buttons[kDrillButton]) {
    drill_msg_.data = 1.0;
  } else {
    drill_msg_.data = 0.0;
  }
  elevator_msg_.data = joystickMsg->axes[kDrillElevationAxis];

  if (elevator_msg_.data > 1.0) {
    elevator_msg_.data = 1.0;
  } else if (elevator_msg_.data < -1.0) {
    elevator_msg_.data = -1.0;
  }

  int value = 4915;

  if (joystickMsg->buttons[kTestServoButton]) {
    value = 5734;
  }

  auto servo_msg = std_msgs::msg::UInt32();
  servo_msg.data = value;
  test_servo_pub_->publish(servo_msg);
  drill_pub_->publish(drill_msg_);
  elevator_pub_->publish(elevator_msg_);
};

void science::declare_parameters() {
  this->declare_parameter("drill_button", 1);
  this->declare_parameter("drill_elevation_axis", 2);
  this->declare_parameter("test_servo_button", 3);
}

void science::load_parameters() {
  this->get_parameter("drill_button", kDrillButton);
  this->get_parameter("drill_elevation_axis", kDrillElevationAxis);
  this->get_parameter("test_servo_button", kTestServoButton);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<science>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}