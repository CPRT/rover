#ifndef ROS_PHOENIX_PHOENIX_MANAGER
#define ROS_PHOENIX_PHOENIX_MANAGER

#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

using namespace rclcpp;

namespace ros_phoenix {

class PhoenixManager : public rclcpp_components::ComponentManager {
 public:
  static std::shared_ptr<PhoenixManager> getInstance(
      std::weak_ptr<rclcpp::Executor> exec);

  static bool instanceCreated();

  static const std::string PARAMETER_PERIOD_MS;
  static const std::string PARAMETER_WATCHDOG_MS;

  rcl_interfaces::msg::SetParametersResult reconfigure(
      const std::vector<rclcpp::Parameter> &params);

 protected:
  PhoenixManager(
      std::weak_ptr<rclcpp::Executor> exec,
      const rclcpp::NodeOptions &node_options = rclcpp::NodeOptions());

 private:
  static std::mutex singleton_mutex_;
  static std::shared_ptr<PhoenixManager> singleton_;

  void feedEnable() const;

  OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_;
  TimerBase::SharedPtr timer_;

  int watchdog_ms_ = -1;
};

}  // namespace ros_phoenix

#endif  // ROS_PHOENIX_PHOENIX_MANAGER