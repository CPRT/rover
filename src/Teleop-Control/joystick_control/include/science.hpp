#ifndef SCIENCE_HPP
#define SCIENCE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>
#include <string>
#include <vector>

class science : public rclcpp::Node {
public:
  science();
  void science_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void register_status(std::shared_ptr<std_msgs::msg::String> status_msg);

private:
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr drill_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr elevator_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr status_sub_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatStatus>::SharedPtr
      device_control_pub_;

  void declare_parameters();
  void load_parameters();

  // Parameters
  int kDrillPowerAxis;
  int kDrillElevationAxis;
  int kServoControlAxis;
  int kServoCount = 8;

  std::vector<int> kServoButtons;
  std::vector<int> kServoPins;
  std::vector<int> kSensorPins;
  std::vector<uint16_t> kSensorConfigValues;
  std::vector<int> kServoFrequencies;
  std::vector<uint16_t> kServoCommandsEncoded;

  bool initialized_;
  std::string current_status_;
  const int ANALOG_SENSOR = 0;
  const int DIGITAL_SENSOR = 1;
};

#endif // SCIENCE_HPP