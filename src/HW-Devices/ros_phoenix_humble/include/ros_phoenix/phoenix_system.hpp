#ifndef ROS_PHOENIX_PHOENIX_SYSTEM
#define ROS_PHOENIX_PHOENIX_SYSTEM

#include <thread>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/base_node.hpp"
#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"

#define Phoenix_No_WPI  // remove WPI dependencies
#include "ctre/Phoenix.h"

namespace ros_phoenix {

class PhoenixSystem : public hardware_interface::SystemInterface {
 public:
  static const std::string PERCENT_OUTPUT;
  static const std::string POSITION;
  static const std::string VELOCITY;

  RCLCPP_SHARED_PTR_DEFINITIONS(PhoenixSystem)

  PhoenixSystem();

  PhoenixSystem(PhoenixSystem &&other) = default;

  ~PhoenixSystem() = default;

  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info);

  std::vector<hardware_interface::StateInterface> export_state_interfaces();

  std::vector<hardware_interface::CommandInterface> export_command_interfaces();

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/);

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/);

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type write(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  struct JointInfo {
    hardware_interface::ComponentInfo info;
    BaseNode::SharedPtr node;
    ros_phoenix::msg::MotorControl::SharedPtr control;
    ros_phoenix::msg::MotorStatus::SharedPtr status;
  };

  static ControlMode str_to_interface(const std::string &str);

  rclcpp::Logger logger_;

  hardware_interface::HardwareInfo info_;

  std::vector<JointInfo> joints_;

  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec_;
  std::thread spin_thread_;

  std::vector<rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr>
      publishers_;
  std::vector<rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr>
      subscribers_;
};

}  // namespace ros_phoenix
#endif  // ROS_PHOENIX_PHOENIX_SYSTEM