#include "cdh_link.hpp"

#include "msgdefs.h"
#include "packet.h"

#include <iomanip>
#include <iostream>

namespace cdh_link {

CdhLink::CdhLink() : Node("cdh_link") {
  system_telemetry_sub_ =
      this->create_subscription<interfaces::msg::SystemTelemetry>(
          "/system_telemetry", 1,
          std::bind(&CdhLink::system_telemetry_cb, this,
                    std::placeholders::_1));
  RCLCPP_INFO(this->get_logger(), "CDH link started");
}

void CdhLink::system_telemetry_cb(
    const interfaces::msg::SystemTelemetry::SharedPtr msg) {
  SysInfoMsg sys_msg{};
  sys_msg.cpu_usage = msg->cpu_usage;
  sys_msg.gpu_usage = msg->gpu_usage;
  sys_msg.mem_usage = msg->mem_usage;
  sys_msg.core_temp = msg->core_temp;

  uint8_t buf[1024];
  SpacePacket::encode(buf, &sys_msg, sizeof(sys_msg), PacketType::TELEMETRY,
                      APID_SYSINFOMSG, 1);

  RCLCPP_INFO(this->get_logger(),
              "%02x %02x %02x %02x %02x %02x %02x %02x %02x %02x", buf[0],
              buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8],
              buf[9]);
}

} // namespace cdh_link

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cdh_link::CdhLink>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}