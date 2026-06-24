#include "interface_manager.h"

InterfaceManagerNode::InterfaceManagerNode(const std::string name,
                                           const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options) {

  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&InterfaceManagerNode::timer_callback, this));

  this->can_status_pub_ =
      this->create_publisher<interfaces::msg::CANStatus>("~/can_status", 10);
}

void InterfaceManagerNode::timer_callback() {
  interfaces::msg::CANStatus msg;
  msg.tx_errors = 6;
  msg.rx_errors = 12;
  if (this->can_status_pub_) {
    this->can_status_pub_->publish(msg);
  }
}