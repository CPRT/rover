#include <array>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_status.hpp"
#include "std_msgs/msg/string.hpp"

class MStates : public rclcpp::Node {
private:
  std::string names[6] = {"base",  "act1",      "act2",
                          "elbow", "wristTilt", "wristTurn"};
  double velocities[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  double positions[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::array<rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr, 6>
      subs;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher2;
  rclcpp::TimerBase::SharedPtr timer_;

public:
  MStates() : Node("m_states") {
    publisher =
        this->create_publisher<std_msgs::msg::String>("mStates/velocity", 10);
    publisher2 =
        this->create_publisher<std_msgs::msg::String>("mStates/position", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
                                     std::bind(&MStates::timer_callback, this));

    for (size_t i = 0; i < subs.size(); ++i) {
      printf("Subscribing to /%s/status\n", names[i].c_str());
      subs[i] = this->create_subscription<ros_phoenix::msg::MotorStatus>(
          "/" + names[i] + "/status", 10,
          [i, this](ros_phoenix::msg::MotorStatus::SharedPtr msg) {
            motor_callback(i, msg);
          });
    }
  }

private:
  void motor_callback(size_t i, ros_phoenix::msg::MotorStatus::SharedPtr msg) {
    this->velocities[i] = msg->velocity;
    this->positions[i] = msg->position;
  }
  void timer_callback() {
    char buffer_vel[256];
    char buffer_pos[256];
    sprintf(buffer_vel,
            "{\"base\":%f,\"act1\":%f,\"act2\":%f,\"elbow\":%f,\"wristTilt\":%"
            "f,\"wristTurn\":%f}",
            velocities[0], velocities[1], velocities[2], velocities[3],
            velocities[4], velocities[5]);
    sprintf(buffer_pos,
            "{\"base\":%f,\"act1\":%f,\"act2\":%f,\"elbow\":%f,\"wristTilt\":%"
            "f,\"wristTurn\":%f}",
            positions[0], positions[1], positions[2], positions[3],
            positions[4], positions[5]);
    std::string data_vel = buffer_vel;
    std::string data_pos = buffer_pos;
    auto message_vel = std_msgs::msg::String();
    auto message_pos = std_msgs::msg::String();
    message_vel.data = data_vel;
    message_pos.data = data_pos;
    publisher2->publish(message_pos);
    publisher->publish(message_vel);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MStates>());
  rclcpp::shutdown();
  return 0;
}
