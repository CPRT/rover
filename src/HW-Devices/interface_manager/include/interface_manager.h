#ifndef INTERFACE_MANAGER_HPP
#define INTERFACE_MANAGER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>

#include "interfaces/msg/can_status.hpp"
#include "interfaces/msg/usb_list.hpp"

#include <asm/types.h>
#include <libusb-1.0/libusb.h>
#include <linux/rtnetlink.h>

extern "C" {
#include "libnetlink.h"
}

struct iplink_req {
  struct nlmsghdr n;
  struct ifinfomsg i;
  char buf[1024];
};

class InterfaceManagerNode : public rclcpp::Node {
public:
  explicit InterfaceManagerNode(
      const std::string name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void timer_callback();
  void send_can(char *name,
                rclcpp::Publisher<interfaces::msg::CANStatus>::SharedPtr pub);
  void send_usb();

  struct rtnl_handle rth;

  libusb_context *ctx;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<interfaces::msg::CANStatus>::SharedPtr can0_status_pub_;
  rclcpp::Publisher<interfaces::msg::CANStatus>::SharedPtr can1_status_pub_;
  rclcpp::Publisher<interfaces::msg::USBList>::SharedPtr usb_list_pub_;
};

#endif // INTERFACE_MANAGER_HPP