#pragma once
#include "hardware_interface/system_interface.hpp"
#include <string>
#include <vector>

namespace gimbal_control {
class MaestroSystem : public hardware_interface::SystemInterface {
public:
  ~MaestroSystem() override;
  hardware_interface::CallbackReturn
  on_init(const hardware_interface::HardwareInfo &) override;
  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &,
                                       const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &,
                                        const rclcpp::Duration &) override;

private:
  struct Servo {
    int channel;
    double lower, upper, center, range, min_us, max_us, direction;
  };
  void close_port();
  bool send_target(size_t index, double position);
  std::vector<Servo> servos_;
  std::vector<double> positions_, commands_;
  std::string port_;
  int fd_{-1}, device_{12}, timeout_ms_{20};
  bool active_{false};
};
} // namespace gimbal_control
