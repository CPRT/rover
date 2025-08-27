#ifndef WHEEL_CONTROL_HPP
#define WHEEL_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"

using namespace ros_phoenix::msg;

/**
 * @brief Enum representing the two possible sides of a wheel: LEFT and RIGHT.
 */
enum class WheelSide { LEFT, RIGHT };

/**
 * @brief The WheelControl class is responsible for controlling a single wheel
 * of the robot. It manages the wheel's velocity and publishes motor control
 * messages to control the wheel.
 */
class WheelControl {
 public:
  /**
   * @brief Struct to hold the status of a wheel, including velocity,
   * temperature, current, and voltages.
   */
  struct Status {
    double velocity;     ///< The current velocity of the wheel.
    double temperature;  ///< The temperature of the motor.
    double current;      ///< The output current to the motor.
    double voltageIn;    ///< The input voltage to the motor.
    double voltageOut;   ///< The output voltage from the motor.
  };

  /**
   * @brief Construct a new WheelControl object.
   * @param wheel_name The name of the wheel (e.g., "frontLeft", "backRight").
   * @param node A pointer to the ROS node for creating publishers.
   * @throws std::invalid_argument if the wheel name doesn't include "Left" or
   * "Right".
   */
  WheelControl(std::string wheel_name, rclcpp::Node* node);

  /**
   * @brief Set the velocity of the wheel.
   * @param value The velocity to set (range depends on the motor control mode).
   */
  void setVelocity(double value);

  /**
   * @brief Publish the current motor control command to the wheel's motor.
   * This sends the velocity command to the motor.
   */
  void send() const;

  /**
   * @brief Get the side of the wheel (left or right).
   * @return WheelSide The side of the wheel.
   */
  WheelSide getWheelSide() const { return side_; }

  /**
   * @brief Get the current velocity of the wheel.
   * @return double The current velocity of the wheel.
   */
  double getVelocity() const { return status_.velocity; }

  /**
   * @brief Set the status of the wheel based on the motor status message.
   * This updates the wheel's internal status with data such as velocity,
   * temperature, etc.
   * @param msg The motor status message received from the motor.
   */
  void setStatus(const MotorStatus::SharedPtr msg);

 private:
  /**
   * @brief The name of the wheel (e.g., "frontLeft").
   */
  std::string name_;

  /**
   * @brief The side of the wheel (LEFT or RIGHT).
   */
  WheelSide side_;

  /**
   * @brief The motor control message used to set the motor parameters.
   */
  MotorControl control_;

  /**
   * @brief The current status of the wheel (e.g., velocity, temperature).
   */
  Status status_;

  /**
   * @brief Pointer to the ROS node for creating publishers.
   */
  rclcpp::Node* node_;

  /**
   * @brief Publisher for sending motor control messages.
   */
  rclcpp::Publisher<MotorControl>::SharedPtr pub_;
};

#endif  // WHEEL_CONTROL_HPP
