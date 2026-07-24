#ifndef SCIENCE_SENSORS_HPP
#define SCIENCE_SENSORS_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int16.hpp>

#include "epoll_event_loop.hpp"
#include "interfaces/msg/dht22.hpp"
#include "interfaces/msg/science_adc.hpp"
#include "interfaces/msg/science_motor.hpp"
#include "interfaces/msg/science_servo.hpp"
#include "socket_can.hpp"

class ScienceNode : public rclcpp::Node {
public:
  ScienceNode(const std::string name,
              const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  bool init(EpollEventLoop *event_loop);
  void deinit();

private:
  void recv_callback(const can_frame &frame);
  void motor_callback();
  void servo_callback();

  inline bool verify_length(const std::string &name, uint8_t expected,
                            uint8_t length);

  uint16_t node_id_;
  SocketCanIntf can_intf_ = SocketCanIntf();

  rclcpp::Publisher<interfaces::msg::ScienceADC>::SharedPtr adc_pub_;
  rclcpp::Publisher<interfaces::msg::DHT22>::SharedPtr temp_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt16>::SharedPtr co2_pub_;

  EpollEvent motor_evt_;
  std::mutex motor_mutex_;
  interfaces::msg::ScienceMotor motor_msg_ = interfaces::msg::ScienceMotor();
  rclcpp::Subscription<interfaces::msg::ScienceMotor>::SharedPtr motor_sub_;

  EpollEvent servo_evt_;
  std::mutex servo_mutex_;
  interfaces::msg::ScienceServo servo_msg_ = interfaces::msg::ScienceServo();
  rclcpp::Subscription<interfaces::msg::ScienceServo>::SharedPtr servo_sub_;
};

#endif // SCIENCE_SENORS_HPP