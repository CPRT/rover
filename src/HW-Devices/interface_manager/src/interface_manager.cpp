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

  this->can_status_pub_ =
      this->create_publisher<interfaces::msg::CANStatus>("/can_status", 10);

  RCLCPP_INFO(get_logger(), "Interface Manager Node started");
}

void InterfaceManagerNode::timer_callback() {
  interfaces::msg::CANStatus msg;
  int fd, len;
  char buf[16];

  fd = open("/sys/class/net/can0/statistics/tx_errors", O_RDONLY);
  len = read(fd, buf, 16);
  buf[len] = '\0';
  msg.tx_errors = strtoul(buf, NULL, 0);
  close(fd);

  fd = open("/sys/class/net/can0/statistics/rx_errors", O_RDONLY);
  len = read(fd, buf, 16);
  buf[len] = '\0';
  msg.rx_errors = strtoul(buf, NULL, 0);
  close(fd);

  fd = open("/sys/class/net/can0/operstate", O_RDONLY);
  len = read(fd, buf, 16);
  buf[len - 1] = '\0';
  msg.status = buf;
  close(fd);

  if (this->can_status_pub_) {
    this->can_status_pub_->publish(msg);
  }
}
