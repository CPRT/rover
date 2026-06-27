#ifndef NODE_STATUS_PUBLISHER_HPP
#define NODE_STATUS_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <vector>

class NodeStatusPublisher : public rclcpp::Node {
public:
  NodeStatusPublisher();

private:
  void publish_nodes();

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // NODE_STATUS_PUBLISHER_HPP