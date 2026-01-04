#include "BaseWrapper.hpp"
BaseWrapper::BaseWrapper(const hardware_interface::ComponentInfo &joint,
                         rclcpp::Node::SharedPtr debug_node)
    : info_(joint), position_(0.0), velocity_(0.0), command_(0.0), freq_(0),
      debug_node_(debug_node), debug_timer_(nullptr) {
  for (const auto &param : joint.parameters) {
    if (param.first == "debug_frequency") {
      freq_ = std::stoi(param.second);
    }
  }
}

void BaseWrapper::configure() {
  if (freq_ > 0) {
    debug_timer_ = debug_node_->create_wall_timer(
        std::chrono::milliseconds(1000 / freq_),
        std::bind(&BaseWrapper::pub_status, this));
  }
}

void BaseWrapper::add_state_interface(
    std::vector<hardware_interface::StateInterface> &state_interfaces) {
  for (const auto &interface : info_.state_interfaces) {
    if (interface.name == hardware_interface::HW_IF_POSITION) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.name, hardware_interface::HW_IF_POSITION, &position_));
    } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
          info_.name, hardware_interface::HW_IF_VELOCITY, &velocity_));
    }
  }
}

void BaseWrapper::add_command_interface(
    std::vector<hardware_interface::CommandInterface> &command_interfaces) {
  for (const auto &interface : info_.command_interfaces) {
    if (interface.name == hardware_interface::HW_IF_POSITION) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.name, hardware_interface::HW_IF_POSITION, &command_));
    } else if (interface.name == hardware_interface::HW_IF_VELOCITY) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
          info_.name, hardware_interface::HW_IF_VELOCITY, &command_));
    }
  }
}