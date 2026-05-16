#include "drill.hpp"

#include <algorithm>
#include <chrono>

drill::drill() : Node("drill_teleop_node") {
  declare_parameters();
  load_parameters();

  drill_pub_ =
      create_publisher<ros_phoenix::msg::MotorControl>(k_drill_cmd_topic_, 10);
  elevator_pub_ = create_publisher<ros_phoenix::msg::MotorControl>(
      k_elevator_cmd_topic_, 10);
  joy_sub_ = create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10,
      std::bind(&drill::drill_control, this, std::placeholders::_1));

  RCLCPP_INFO(
      this->get_logger(),
      "Drill teleop started: drill axis=%d, elevation axis=%d -> '%s' / '%s'",
      k_drill_power_axis_, k_drill_elevation_axis_, k_drill_cmd_topic_.c_str(),
      k_elevator_cmd_topic_.c_str());
}

void drill::drill_control(std::shared_ptr<sensor_msgs::msg::Joy> joystick_msg) {
  const size_t n_axes = joystick_msg->axes.size();
  if (static_cast<size_t>(k_drill_power_axis_) >= n_axes ||
      static_cast<size_t>(k_drill_elevation_axis_) >= n_axes) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Joy message has %zu axes; need indices %d and %d",
                         n_axes, k_drill_power_axis_, k_drill_elevation_axis_);
    return;
  }

  const double power_axis = joystick_msg->axes[k_drill_power_axis_];
  const double elev_axis = joystick_msg->axes[k_drill_elevation_axis_];

  const double drill_t = 0.5 * (power_axis + 1.0);
  const double drill_value = std::clamp(drill_t * k_max_drill_duty_, 0.0, 1.0);
  const double elev_value =
      std::clamp(elev_axis * k_max_elevator_duty_, -1.0, 1.0);

  auto drill_msg = ros_phoenix::msg::MotorControl();
  drill_msg.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
  drill_msg.value = drill_value;

  auto elev_msg = ros_phoenix::msg::MotorControl();
  elev_msg.mode = ros_phoenix::msg::MotorControl::PERCENT_OUTPUT;
  elev_msg.value = elev_value;

  drill_pub_->publish(drill_msg);
  elevator_pub_->publish(elev_msg);
}

void drill::declare_parameters() {
  declare_parameter("drill_power_axis", 3);
  declare_parameter("drill_elevation_axis", 1);
  declare_parameter("max_drill_duty", 1.0);
  declare_parameter("max_elevator_duty", 1.0);
  declare_parameter("drill_cmd_topic", "/drill/set");
  declare_parameter("elevator_cmd_topic", "/elevator/set");
}

void drill::load_parameters() {
  this->get_parameter("drill_power_axis", k_drill_power_axis_);
  this->get_parameter("drill_elevation_axis", k_drill_elevation_axis_);
  this->get_parameter("max_drill_duty", k_max_drill_duty_);
  this->get_parameter("max_elevator_duty", k_max_elevator_duty_);
  this->get_parameter("drill_cmd_topic", k_drill_cmd_topic_);
  this->get_parameter("elevator_cmd_topic", k_elevator_cmd_topic_);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<drill>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
