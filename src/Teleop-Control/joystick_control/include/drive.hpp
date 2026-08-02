#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/video_preset.hpp"
#include "interfaces/srv/video_out.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include <interfaces/msg/video_presets.hpp>

class drive : public rclcpp::Node {
public:
  drive();
  void drive_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_y_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_x_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr servo_m_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr drive_throttle_sub_;
  rclcpp::Subscription<interfaces::msg::VideoPresets>::SharedPtr presets_sub_;
  rclcpp::Client<interfaces::srv::VideoOut>::SharedPtr camera_client_;
  std::vector<interfaces::msg::VideoPreset> video_carousell_;
  size_t video_carousell_idx_;

  void camera_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void drive_throttle_cb(const std_msgs::msg::Float32::SharedPtr throttle_msg);
  void declare_parameters();
  void load_parameters();
  void presets_cb(const interfaces::msg::VideoPresets::SharedPtr presets_msg);

  // Parameters
  double kMaxLinear;
  double kMaxAngular;
  int kForwardAxis;
  int kYawAxis;
  int kStrafeAxis;
  int kServoYAxis;
  int kServoXAxis;
  int kServoHomeButton;
  int kMastLeftButton;
  int kMastRightButton;
  int kLockTurnBut;
  int kCamLeftBut;
  int kCamRightBut;
  double kServoIncrement;
  double kServoMin;
  double kServoMax;
  double kJoyDeadzone;
  double kDefaultServoX;
  double kDefaultServoY;

  bool initialized_;
  double servo_y_;
  double servo_x_;
  double servo_mast_;
  float drive_throttle_;

  bool cam_debounce_;
};

#endif // DRIVE_HPP