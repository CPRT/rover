/**
 * @file Mode.hpp
 * @brief Defines the Mode class for handling joystick input in different modes.
 */

#ifndef MODE_HPP
#define MODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

/**
 * @class Mode
 * @brief Abstract base class for different joystick control modes.
 */
class Mode {
 public:
  /**
   * @brief Constructor for the Mode class.
   * @param name The name of the mode.
   * @param node Pointer to the ROS2 node.
   */
  Mode(std::string name, rclcpp::Node* node) : name_(name), node_(node) {}
  virtual ~Mode() = default;

  /**
   * @brief Pure virtual function to process joystick input.
   * @param joystickMsg Shared pointer to the joystick message.
   */
  virtual void processJoystickInput(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) = 0;

  /**
   * @brief Get the name of the mode.
   * @return The name of the mode.
   */
  std::string getName() { return name_; }

 protected:
  std::string name_;    ///< The name of the mode.
  rclcpp::Node* node_;  ///< Pointer to the ROS2 node.
};

#endif  // MODE_HPP