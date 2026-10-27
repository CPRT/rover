#include "cdh_node.hpp"

#include "cdh_link.hpp"
#include "msgdefs.h"

#include <iomanip>
#include <iostream>

#include <arpa/inet.h>
#include <sys/socket.h>

namespace cdh_link {

struct TopicIdx {
  std::string topic;
  uint8_t idx;
};

TopicIdx drive_motors[8] = {
    {"/Left_front_wheel_joint", 0},     {"/Right_front_wheel_joint", 1},
    {"/Left_rear_wheel_joint", 2},      {"/Right_rear_wheel_joint", 3},
    {"/Left_front_wheel_arm_joint", 4}, {"/Right_front_wheel_arm_joint", 5},
    {"/Left_rear_wheel_arm_joint", 6},  {"/Right_rear_wheel_arm_joint", 7}};

CdhNode::CdhNode()
    : Node("cdh_node"),
      link(this->get_logger(), "Link 1",
           std::bind(&CdhNode::recv_link1, this, std::placeholders::_1,
                     std::placeholders::_2, std::placeholders::_3,
                     std::placeholders::_4)) {
  this->declareParameters();
  this->loadParameters();

  system_telemetry_sub_ =
      this->create_subscription<interfaces::msg::SystemTelemetry>(
          "/system_telemetry", 1,
          std::bind(&CdhNode::system_telemetry_cb, this,
                    std::placeholders::_1));

  for (TopicIdx motor : drive_motors) {
    drive_joint_subs_[motor.idx] =
        this->create_subscription<ros_phoenix::msg::MotorStatus>(
            motor.topic + "/status", 1,
            [this, motor_idx = motor.idx](
                const ros_phoenix::msg::MotorStatus::SharedPtr msg) {
              this->drive_joint_cb(msg, motor_idx);
            });
  }

  joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/drive/joy",
                                                           rclcpp::QoS(1));

  this->link.open(kTmIp, kTmPort, kTcPort);
  timer_ =
      this->create_wall_timer(std::chrono::duration<double>(1.0), [this]() {
        this->link.sendTlm(drive_status_, APID_DRIVESTATUSMSG);
        drive_status_.seq++;
      });
  RCLCPP_INFO(this->get_logger(), "CDH node started");
}

void CdhNode::system_telemetry_cb(
    const interfaces::msg::SystemTelemetry::SharedPtr msg) {
  sys_info_.msg.cpu_usage = msg->cpu_usage;
  sys_info_.msg.gpu_usage = msg->gpu_usage;
  sys_info_.msg.mem_usage = msg->mem_usage;
  sys_info_.msg.core_temp = msg->core_temp;
  this->link.sendTlm(sys_info_, APID_SYSINFOMSG);
  sys_info_.seq++;
}

void CdhNode::drive_joint_cb(const ros_phoenix::msg::MotorStatus::SharedPtr msg,
                             uint8_t idx) {
  drive_status_.msg.motors[idx].bus_voltage = msg->bus_voltage;
  drive_status_.msg.motors[idx].current = msg->output_current;
  drive_status_.msg.motors[idx].errors = msg->active_errors;
  drive_status_.msg.motors[idx].position = msg->position;
  drive_status_.msg.motors[idx].velocity = msg->velocity;
}

void CdhNode::recv_link1(uint8_t *payload, PacketType type, uint16_t apid,
                         uint16_t seq) {
  switch (apid) {
  case APID_DRIVECTRLCMD: {
    DriveCtrlCmd *drive_cmd = reinterpret_cast<DriveCtrlCmd *>(payload);
    sensor_msgs::msg::Joy joy_msg;
    joy_msg.axes.push_back(drive_cmd->x);
    joy_msg.axes.push_back(drive_cmd->y);
    joy_msg.axes.push_back(drive_cmd->yaw);
    joy_msg.axes.push_back(drive_cmd->throttle);
    if (joy_pub_) {
      joy_pub_->publish(joy_msg);
    }
  } break;

  default:
    break;
  }
}

void CdhNode::declareParameters() {
  this->declare_parameter("tm_port", 1234);
  this->declare_parameter("tm_ip", std::string("127.0.0.1"));
  this->declare_parameter("tc_port", 1235);
}

void CdhNode::loadParameters() {
  this->get_parameter("tm_port", kTmPort);
  this->get_parameter("tm_ip", kTmIp);
  this->get_parameter("tc_port", kTcPort);
}

} // namespace cdh_link

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<cdh_link::CdhNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}