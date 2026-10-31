#ifndef CDH_NODE_HPP_
#define CDH_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/system_telemetry.hpp"
#include "ros_phoenix/msg/motor_status.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

#include "cdh_link.hpp"
#include "msgdefs.h"
#include "packet.h"

namespace cdh_link {

class CdhNode : public rclcpp::Node {
public:
  explicit CdhNode();

  void recv_link1(uint8_t *payload, PacketType type, uint16_t apid,
                  uint16_t seq);
  void recv_link2(uint8_t *payload, PacketType type, uint16_t apid,
                  uint16_t seq);

private:
  void declareParameters();
  void loadParameters();

  void
  system_telemetry_cb(const interfaces::msg::SystemTelemetry::SharedPtr msg);
  void drive_joint_cb(const ros_phoenix::msg::MotorStatus::SharedPtr msg,
                      uint8_t idx);

  rclcpp::Subscription<interfaces::msg::SystemTelemetry>::SharedPtr
      system_telemetry_sub_;
  rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr
      drive_joint_subs_[8];

  void processCmd(uint8_t *payload, uint16_t apid, uint16_t seq);
  template <typename T> void sendTlm(TlmMsg<T> msg, uint16_t apid);

  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr throttle_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  CdhLink link1;
  CdhLink link2;

  uint16_t kTm1Port;
  std::string kTm1Ip;
  uint16_t kTc1Port;
  uint16_t kTm2Port;
  std::string kTm2Ip;
  uint16_t kTc2Port;

  TlmMsg<DriveStatusMsg> drive_status_;
  TlmMsg<NavInfoMsg> nav_info_;
  TlmMsg<SysInfoMsg> sys_info_;
};

} // namespace cdh_link

#endif // CDH_NODE_HPP_