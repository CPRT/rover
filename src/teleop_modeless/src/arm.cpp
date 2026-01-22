#include "arm.hpp"

arm::arm() : Node("arm_node") {
  joy_sub = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&arm::arm_control, this, std::placeholders::_1));
  joint_pub = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", 10);

  joint_msg_ = control_msgs::msg::JointJog();
  joint_msg_.joint_names = {"Joint_1", "Joint_2", "Joint_3",
                            "Joint_4", "Joint_5", "Joint_6"};
}