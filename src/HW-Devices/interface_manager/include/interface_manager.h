#ifndef INTERFACE_MANAGER_HPP
#define INTERFACE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "interfaces/msg/can_status.hpp"

class InterfaceManagerNode : public rclcpp::Node {
public:
  explicit InterfaceManagerNode(
      const std::string name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void timer_callback();
  void send_can(char *name,
                rclcpp::Publisher<interfaces::msg::CANStatus>::SharedPtr pub);

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<interfaces::msg::CANStatus>::SharedPtr can0_status_pub_;
  rclcpp::Publisher<interfaces::msg::CANStatus>::SharedPtr can1_status_pub_;
};

#endif // INTERFACE_MANAGER_HPP