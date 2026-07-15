#ifndef NODE_STATUS_PUBLISHER_HPP
#define NODE_STATUS_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <vector>

#include "interfaces/msg/node_list.hpp"

class NodeStatusPublisher : public rclcpp::Node {
public:
  NodeStatusPublisher();

private:
  void publish_nodes();

  double frequency_;

  rclcpp::Publisher<interfaces::msg::NodeList>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // NODE_STATUS_PUBLISHER_HPP