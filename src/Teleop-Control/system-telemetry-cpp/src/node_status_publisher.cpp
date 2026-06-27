#include "../include/system-telemetry-cpp/node_status_publisher.hpp"

#include <rclcpp/qos.hpp>
#include <sstream>
#include <vector>

NodeStatusPublisher::NodeStatusPublisher() : Node("node_status_publisher") {

  rclcpp::QoS qos(10);
  qos.transient_local();

  publisher_ =
      this->create_publisher<std_msgs::msg::String>("/system/nodes", qos);

  timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&NodeStatusPublisher::publish_nodes, this));

  RCLCPP_INFO(this->get_logger(), "NodeStatusPublisher started (ROS-native)");
}

void NodeStatusPublisher::publish_nodes() {
  // Get all nodes in the ROS graph (native API)
  std::vector<std::string> nodes = this->get_node_names();

  // Build JSON
  std::ostringstream json;
  json << "{\"nodes\": [";

  for (size_t i = 0; i < nodes.size(); ++i) {
    json << "\"" << nodes[i] << "\"";
    if (i + 1 < nodes.size())
      json << ", ";
  }

  json << "]}";

  std_msgs::msg::String msg;
  msg.data = json.str();

  publisher_->publish(msg);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NodeStatusPublisher>());
  rclcpp::shutdown();
  return 0;
}