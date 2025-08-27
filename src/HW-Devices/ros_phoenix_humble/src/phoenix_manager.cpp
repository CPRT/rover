#include "ros_phoenix/phoenix_manager.hpp"

#define Phoenix_No_WPI  // remove WPI dependencies
#include <functional>
#include <iostream>

#include "ctre/phoenix/cci/Unmanaged_CCI.h"
#include "ctre/phoenix/platform/can/PlatformCAN.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"

namespace ros_phoenix {

std::mutex PhoenixManager::singleton_mutex_;
std::shared_ptr<PhoenixManager> PhoenixManager::singleton_;

const std::string PhoenixManager::PARAMETER_PERIOD_MS = "period_ms";
const std::string PhoenixManager::PARAMETER_WATCHDOG_MS = "watchdog_ms";

std::shared_ptr<PhoenixManager> PhoenixManager::getInstance(
    std::weak_ptr<rclcpp::Executor> exec) {
  std::lock_guard<std::mutex> lock_guard(PhoenixManager::singleton_mutex_);
  if (!PhoenixManager::singleton_) {
    PhoenixManager::singleton_ =
        std::shared_ptr<PhoenixManager>(new PhoenixManager(exec));
  } else {
    throw new std::runtime_error("Phoenix manager already exists!");
  }
  return singleton_;
}

bool PhoenixManager::instanceCreated() {
  return bool(PhoenixManager::singleton_);
}

PhoenixManager::PhoenixManager(std::weak_ptr<rclcpp::Executor> exec,
                               const rclcpp::NodeOptions &options)
    : ComponentManager(exec, "PhoenixManager", options) {
  this->declare_parameter<int>(PARAMETER_PERIOD_MS, 50);
  this->declare_parameter<int>(PARAMETER_WATCHDOG_MS, 200);

  this->reconfigure({this->get_parameter(PARAMETER_PERIOD_MS),
                     this->get_parameter(PARAMETER_WATCHDOG_MS)});

  this->set_parameters_callback_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        return this->reconfigure(params);
      });

  c_SetPhoenixDiagnosticsStartTime(1);
}

void PhoenixManager::feedEnable() const {
  ctre::phoenix::unmanaged::Unmanaged::FeedEnable(this->watchdog_ms_);
}

rcl_interfaces::msg::SetParametersResult PhoenixManager::reconfigure(
    const std::vector<rclcpp::Parameter> &params) {
  for (const auto &param : params) {
    if (param.get_name() == PARAMETER_PERIOD_MS) {
      this->timer_ =
          this->create_wall_timer(std::chrono::milliseconds(param.as_int()),
                                  std::bind(&PhoenixManager::feedEnable, this));
    } else if (param.get_name() == PARAMETER_WATCHDOG_MS) {
      this->watchdog_ms_ = param.as_int();
    }
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace ros_phoenix