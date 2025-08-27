#include "rover_arm.hpp"

#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/can/PlatformCAN.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

namespace ros2_control_rover_arm {

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  debug_node_ = std::make_shared<rclcpp::Node>("rover_arm_debug_node");

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    try {
      auto controller = std::make_shared<TalonSRXWrapper>(joint, debug_node_);
      controller->configure();
      controllers_.push_back(controller);
    } catch (const std::exception &e) {
      RCLCPP_FATAL(rclcpp::get_logger("RoverArmHardwareInterface"),
                   "Failed to initialize controller for joint '%s': %s",
                   joint.name.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("RoverArmHardwareInterface"),
              "Successfully initialized controllers!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  c_SetPhoenixDiagnosticsStartTime(1);
  RCLCPP_INFO(rclcpp::get_logger("RoverArmHardwareInterface"),
              "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RoverArmHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto &controller : controllers_) {
    controller->add_state_interface(state_interfaces);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RoverArmHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto &controller : controllers_) {
    controller->add_command_interface(command_interfaces);
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("RoverArmHardwareInterface"),
              "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("RoverArmHardwareInterface"),
              "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverArmHardwareInterface::read(
    const rclcpp::Time &, const rclcpp::Duration &) {
  for (auto &controller : controllers_) {
    controller->read();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverArmHardwareInterface::write(
    const rclcpp::Time &, const rclcpp::Duration &) {
  for (auto &controller : controllers_) {
    controller->write();
  }
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(100);
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_rover_arm

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_rover_arm::RoverArmHardwareInterface,
                       hardware_interface::SystemInterface)
