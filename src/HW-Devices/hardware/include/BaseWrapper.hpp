#pragma once

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

/**
 * @brief Abstract base wrapper for motor controller interfaces.
 *
 * This class provides the common interface structure for hardware wrappers
 * used with ros2_control. Derived classes must implement the read() and
 * write() functions and may override configuration or interface registration
 * methods as needed.
 */
class BaseWrapper {
public:
  /// @brief Virtual destructor.
  virtual ~BaseWrapper() = default;

  /**
   * @brief Write the command value to the hardware.
   *
   * This is typically used to send actuator commands (e.g., motor output).
   * Must be implemented by derived classes.
   */
  virtual void write() = 0;

  /**
   * @brief Read the current hardware state.
   *
   * This should update position, velocity, effort, or other state values.
   * Must be implemented by derived classes.
   */
  virtual void read() = 0;

  /**
   * @brief Register state interfaces for this joint.
   *
   * Derived classes may override this to expose additional state interfaces.
   *
   * @param state_interfaces Vector to append state interfaces into.
   */
  virtual void add_state_interface(
      std::vector<hardware_interface::StateInterface> &state_interfaces);

  /**
   * @brief Register command interfaces for this joint.
   *
   * Derived classes may override this to expose additional command interfaces.
   *
   * @param command_interfaces Vector to append command interfaces into.
   */
  virtual void add_command_interface(
      std::vector<hardware_interface::CommandInterface> &command_interfaces);

  /**
   * @brief Publish hardware telemetry or diagnostic information.
   *
   * Optional for derived classes. Called periodically when debug is enabled.
   */
  virtual void pub_status() const {}

  /**
   * @brief Configure common elements for all motor controllers.
   *
   * Derived classes should call this function first in their own configure()
   * implementation, then apply additional configuration as required.
   */
  virtual void configure();
  /**
   * @brief Activate motor controllers.
   *
   * Optional for derived classes.
   */
  virtual void activate() {};

protected:
  /**
   * @brief Construct a BaseWrapper.
   *
   * @param joint Joint configuration provided by ros2_control.
   * @param debug_node Node used for optional debug or telemetry publishing.
   */
  BaseWrapper(const hardware_interface::ComponentInfo &joint,
              rclcpp::Node::SharedPtr debug_node);

  // Common configuration info from ros2_control
  const hardware_interface::ComponentInfo info_;

  // Common interfaces
  double position_;
  double velocity_;
  double command_;

  // ROS 2 debug node and timer
  int freq_;
  rclcpp::Node::SharedPtr debug_node_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
};
