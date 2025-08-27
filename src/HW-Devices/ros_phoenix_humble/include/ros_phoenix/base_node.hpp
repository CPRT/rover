#ifndef ROS_PHOENIX_MOTOR_CONTROLLER
#define ROS_PHOENIX_MOTOR_CONTROLLER

#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"

// Constants for input_type parameter.
#define ANALOG 1
#define ABSOLUTE 2
#define RELATIVE 3

using namespace rclcpp;
using namespace ros_phoenix::msg;

namespace ros_phoenix {

class BaseNode : public Node {
 public:
  struct Parameter {
    static const std::string ID;
    static const std::string INTERFACE;
  };

  RCLCPP_SHARED_PTR_DEFINITIONS(BaseNode)

  BaseNode(const std::string &name, const NodeOptions &options = NodeOptions());

  virtual ~BaseNode();

  virtual MotorStatus::SharedPtr status() = 0;

  virtual void set(MotorControl::SharedPtr control_msg);

  virtual rcl_interfaces::msg::SetParametersResult reconfigure(
      const std::vector<rclcpp::Parameter> &params);

 protected:
  virtual void configure() = 0;

  virtual void onTimer();

  int id_;
  std::string interface_;

  int follow_id_;
  double sensor_multiplier_ = 1.0;
  double sensor_offset_ = 0.0;

  bool configured_ = false;
  std::mutex config_mutex_;

 private:
  int watchdog_ms_;
  int period_ms_;

  OnSetParametersCallbackHandle::SharedPtr set_parameters_callback_;

  std::shared_ptr<std::thread> config_thread_;

  rclcpp::TimerBase::SharedPtr timer_;
  bool watchdog_warned_ = true;
  rclcpp::Time last_update_;
};

}  // namespace ros_phoenix

#endif  // ROS_PHOENIX_MOTOR_CONTROLLER
