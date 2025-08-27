#ifndef JOYSTICK_CONTROL__ARM_HELPERS_HPP_
#define JOYSTICK_CONTROL__ARM_HELPERS_HPP_

#include "rclcpp/rclcpp.hpp"

namespace ArmHelpers {
bool start_moveit_servo(rclcpp::Node* node, int attempts = 3);
}

#endif