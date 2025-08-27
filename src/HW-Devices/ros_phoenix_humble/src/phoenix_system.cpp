#include "ros_phoenix/phoenix_system.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "ros_phoenix/phoenix_manager.hpp"
#include "ros_phoenix/phoenix_nodes.hpp"

hardware_interface::CallbackReturn set_parameters(
    const std::unordered_map<std::string, std::string> parameters,
    rclcpp::Node::SharedPtr node) {
  for (const auto &p : parameters) {
    std::string name(p.first);
    std::string value(p.second);

    if (node->has_parameter(name)) {
      auto param = node->get_parameter(name);
      switch (param.get_type()) {
        case ParameterType::PARAMETER_STRING:
          node->set_parameter(Parameter(name, value));
          break;
        case ParameterType::PARAMETER_INTEGER:
          node->set_parameter(Parameter(name, std::stoi(value)));
          break;
        case ParameterType::PARAMETER_DOUBLE:
          node->set_parameter(Parameter(name, std::stod(value)));
          break;
        case ParameterType::PARAMETER_BOOL:
          if (value == "true") {
            node->set_parameter(Parameter(name, true));
          } else if (value == "false") {
            node->set_parameter(Parameter(name, false));
          } else {
            RCLCPP_FATAL(
                node->get_logger(),
                "Boolean parameter '%s' must be either 'true' or 'false'",
                name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
          }
          break;
        default:
          RCLCPP_FATAL(node->get_logger(), "Unsupported parameter type: %s",
                       param.get_type_name().c_str());
          return hardware_interface::CallbackReturn::ERROR;
      }
    } else {
      RCLCPP_FATAL(node->get_logger(), "Unknown parameter '%s' for node '%s'",
                   name.c_str(), node->get_name());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

namespace ros_phoenix {

const std::string PhoenixSystem::PERCENT_OUTPUT = "percent_output";
const std::string PhoenixSystem::POSITION = hardware_interface::HW_IF_POSITION;
const std::string PhoenixSystem::VELOCITY = hardware_interface::HW_IF_VELOCITY;

BaseNode::SharedPtr create_node(const std::string &name,
                                const std::string &type) {
  if (type == "ros_phoenix::TalonFX") {
    return std::shared_ptr<TalonFXNode>(new TalonFXNode(name));
  } else if (type == "ros_phoenix::TalonSRX") {
    return std::shared_ptr<TalonSRXNode>(new TalonSRXNode(name));
  } else if (type == "ros_phoenix::VictorSPX") {
    return std::shared_ptr<VictorSPXNode>(new VictorSPXNode(name));
  }

  return nullptr;
}

ControlMode PhoenixSystem::str_to_interface(const std::string &str) {
  if (str == PhoenixSystem::PERCENT_OUTPUT) {
    return ControlMode::PercentOutput;

  } else if (str == PhoenixSystem::POSITION) {
    return ControlMode::Position;

  } else if (str == PhoenixSystem::VELOCITY) {
    return ControlMode::Velocity;

  } else {
    return ControlMode::Disabled;
  }
}

PhoenixSystem::PhoenixSystem() : logger_(rclcpp::get_logger("PhoenixSystem")) {}

hardware_interface::CallbackReturn PhoenixSystem::on_init(
    const hardware_interface::HardwareInfo &info) {
  this->logger_ = rclcpp::get_logger(info.name);
  this->exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  std::shared_ptr<ros_phoenix::PhoenixManager> phoenix_manager = nullptr;
  try {
    phoenix_manager = ros_phoenix::PhoenixManager::getInstance(this->exec_);
  } catch (const std::runtime_error &exec) {
    RCLCPP_FATAL(this->logger_,
                 "Multiple instance of PhoenixSystem were "
                 "detected. Only one per process is allowed!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto rc = set_parameters(info.hardware_parameters, phoenix_manager);
  if (rc != hardware_interface::CallbackReturn::SUCCESS) return rc;

  this->exec_->add_node(phoenix_manager);

  for (auto joint : info.joints) {
    auto type_param = joint.parameters.find("type");
    if (type_param == joint.parameters.end()) {
      RCLCPP_FATAL(phoenix_manager->get_logger(),
                   "Joint '%s' is missing required parameter 'type'",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    auto type_name = (*type_param).second;
    auto node = create_node(joint.name, type_name);
    if (!node) {
      RCLCPP_FATAL(phoenix_manager->get_logger(),
                   "Joint '%s' is of invalid type '%s'", joint.name.c_str(),
                   type_name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    auto parameters = joint.parameters;
    parameters.erase("type");
    auto rc = set_parameters(parameters, node);
    if (rc != hardware_interface::CallbackReturn::SUCCESS) return rc;

    this->exec_->add_node(node);

    auto control = std::make_shared<ros_phoenix::msg::MotorControl>();
    auto status = std::make_shared<ros_phoenix::msg::MotorStatus>();

    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(this->logger_,
                   "Joint '%s' has %d command interfaces. Expected 1.",
                   joint.name.c_str(),
                   static_cast<int>(joint.command_interfaces.size()));
      return hardware_interface::CallbackReturn::ERROR;
    }
    ControlMode cmd_interface =
        str_to_interface(joint.command_interfaces[0].name);
    if (cmd_interface == ControlMode::Disabled) {
      RCLCPP_FATAL(
          this->logger_, "Joint '%s' has an invalid command interface: %s",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    control->mode = static_cast<int>(cmd_interface);

    for (auto &state_intf : joint.state_interfaces) {
      if (str_to_interface(state_intf.name) == ControlMode::Disabled) {
        RCLCPP_FATAL(this->logger_,
                     "Joint '%s' has an invalid state interface: %s",
                     joint.name.c_str(), state_intf.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    this->joints_.push_back({joint, node, control, status});
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
PhoenixSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto &joint : this->joints_) {
    for (auto &state_inter : joint.info.state_interfaces) {
      if (state_inter.name == PhoenixSystem::PERCENT_OUTPUT) {
        state_interfaces.emplace_back(joint.info.name,
                                      PhoenixSystem::PERCENT_OUTPUT,
                                      &(joint.status->output_percent));

      } else if (state_inter.name == PhoenixSystem::POSITION) {
        state_interfaces.emplace_back(joint.info.name, PhoenixSystem::POSITION,
                                      &(joint.status->position));

      } else if (state_inter.name == PhoenixSystem::VELOCITY) {
        state_interfaces.emplace_back(joint.info.name, PhoenixSystem::VELOCITY,
                                      &(joint.status->velocity));
      }
    }
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
PhoenixSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto &joint : this->joints_) {
    command_interfaces.emplace_back(joint.info.name,
                                    joint.info.command_interfaces[0].name,
                                    &(joint.control->value));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn PhoenixSystem::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  this->spin_thread_ = std::thread([this] { this->exec_->spin(); });
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PhoenixSystem::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  this->exec_->cancel();
  this->spin_thread_.join();

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PhoenixSystem::read(const rclcpp::Time &,
                                                    const rclcpp::Duration &) {
  for (auto &joint : this->joints_) {
    *(joint.status) = *(joint.node->status());
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PhoenixSystem::write(const rclcpp::Time &,
                                                     const rclcpp::Duration &) {
  for (auto &joint : this->joints_) {
    joint.node->set(joint.control);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace ros_phoenix

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ros_phoenix::PhoenixSystem,
                       hardware_interface::SystemInterface)