#ifndef JOYSTICK_CONTROL__SCIENCEMODE_HPP_
#define JOYSTICK_CONTROL__SCIENCEMODE_HPP_

#include <cmath>

#include "Mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/srv/move_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"
#include "std_srvs/srv/set_bool.hpp"

class ScienceMode : public Mode {
public:
  ScienceMode(rclcpp::Node *node);

  void processJoystickInput(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;

  static void declareParameters(rclcpp::Node *node);

private:
  void handlePlatform(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;
  void handleDrill(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;
  void
  handleMicroscope(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;
  void
  handlePanoramic(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;
  void handleSoilCollection(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;

  void setServoPosition(std::string name, double position) const;
  void toggleLights() const;

  void loadParameters();

  // constant for radians
  const double rad_multiplier = M_PI / 180;

  int8_t kPlatformAxis;
  int8_t kDrillButton;
  int8_t kMicroscopeAxis;
  int8_t kThrottleAxis;
  int8_t kCollectionButton;
  int8_t kCancelCollectionButton;
  int8_t kSoilTestButton;
  int8_t kSoilLockButton;
  int8_t kMicroscopeLightButton;
  int8_t kCollectionSample;
  int16_t kCollectionOpen;
  int16_t kCollectionClose;
  int16_t kCollectionLock;
  std::string collectionServo;
  std::string microscopeServo;

  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr platform_pub_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr drill_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr led_client_;

  // Map that maps motor names to publishers
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr>
      motor_pubs;
};

#endif
