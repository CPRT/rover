#include "interface_manager.h"

#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

InterfaceManagerNode::InterfaceManagerNode(const std::string name,
                                           const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options) {

  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000),
      std::bind(&InterfaceManagerNode::timer_callback, this));

  this->can0_status_pub_ =
      this->create_publisher<interfaces::msg::CANStatus>("/can0_status", 10);

  this->can1_status_pub_ =
      this->create_publisher<interfaces::msg::CANStatus>("/can1_status", 10);

  RCLCPP_INFO(get_logger(), "Interface Manager Node started");
}

void InterfaceManagerNode::timer_callback() {
  this->send_can("can0", this->can0_status_pub_);
  this->send_can("can1", this->can1_status_pub_);
}

void InterfaceManagerNode::send_can(
    char *name, rclcpp::Publisher<interfaces::msg::CANStatus>::SharedPtr pub) {
  interfaces::msg::CANStatus msg;
  int fd, len;
  char buf[16], loc[64];

  sprintf(loc, "/sys/class/net/%s/statistics/tx_errors", name);
  fd = open(loc, O_RDONLY);
  len = read(fd, buf, 16);
  buf[len] = '\0';
  msg.tx_errors = strtoul(buf, NULL, 0);
  close(fd);

  sprintf(loc, "/sys/class/net/%s/statistics/rx_errors", name);
  fd = open(loc, O_RDONLY);
  len = read(fd, buf, 16);
  buf[len] = '\0';
  msg.rx_errors = strtoul(buf, NULL, 0);
  close(fd);

  sprintf(loc, "/sys/class/net/%s/operstate", name);
  fd = open(loc, O_RDONLY);
  len = read(fd, buf, 16);
  buf[len - 1] = '\0';
  msg.status = buf;
  close(fd);

  if (pub) {
    pub->publish(msg);
  }
}
