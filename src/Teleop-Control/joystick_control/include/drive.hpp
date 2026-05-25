#ifndef DRIVE_HPP
#define DRIVE_HPP

#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/msg/video_source.hpp"
#include "interfaces/srv/video_out.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

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
  rclcpp::Client<interfaces::srv::VideoOut>::SharedPtr camera_client_;
  std::vector<interfaces::srv::VideoOut::Request> video_carousell_;
  size_t video_carousell_idx_;

  void camera_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void declare_parameters();
  void load_parameters();
  void setCarousell();

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

  bool cam_debounce_;
};

#endif // DRIVE_HPP