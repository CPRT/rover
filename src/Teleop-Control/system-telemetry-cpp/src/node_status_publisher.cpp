#include "../include/system-telemetry-cpp/node_status_publisher.hpp"

#include <rclcpp/qos.hpp>
#include <sstream>
#include <vector>

NodeStatusPublisher::NodeStatusPublisher()
    : Node("node_status_publisher"),
      frequency_(this->declare_parameter<double>("frequency", 1.0)) {

  rclcpp::QoS qos(10);
  qos.transient_local();

  publisher_ =
      this->create_publisher<interfaces::msg::NodeList>("/system/nodes", qos);

  topics_service_ = this->create_service<interfaces::srv::GetTopics>(
      "/system/get_topics",
      std::bind(&NodeStatusPublisher::get_topics, this, std::placeholders::_1,
                std::placeholders::_2));

  const auto period = std::chrono::duration<double>(1.0 / frequency_);
  timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&NodeStatusPublisher::publish_nodes, this));

  RCLCPP_INFO(this->get_logger(), "NodeStatusPublisher started (ROS-native)");
}

void NodeStatusPublisher::publish_nodes() {
  interfaces::msg::NodeList msg;
  msg.nodes = this->get_node_names();

  publisher_->publish(msg);
}

void NodeStatusPublisher::get_topics(
    const std::shared_ptr<interfaces::srv::GetTopics::Request> request,
    std::shared_ptr<interfaces::srv::GetTopics::Response> response) {
  (void)request;

  const auto topics_and_types = this->get_topic_names_and_types();

  for (const auto &[topic, types] : topics_and_types) {
    for (const auto &type : types) {
      response->topics.push_back(topic);
      response->types.push_back(type);
    }
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodeStatusPublisher>());
  rclcpp::shutdown();
  return 0;
}