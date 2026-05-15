#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace compressed_telemetry_cpp {

class TFRelayNode : public rclcpp::Node {
public:
  explicit TFRelayNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("tf_relay", options) {

    // --- The QoS Bridge ---
    // Listen to the spotty radio link using Best Effort
    auto sub_qos = rclcpp::QoS(10).best_effort().durability_volatile();

    // Publish to the local basestation network using Reliable so RViz is happy
    auto pub_qos = rclcpp::QoS(10).reliable().durability_volatile();

    tf_pub_ =
        this->create_publisher<tf2_msgs::msg::TFMessage>("/viz/tf", pub_qos);

    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/telemetry/tf", sub_qos,
        [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
          // Pass-through exactly what we received
          tf_pub_->publish(*msg);
        });

    RCLCPP_INFO(this->get_logger(), "TF Relay Started: Bridging /telemetry/tf "
                                    "(Best Effort) -> /tf (Reliable)");
  }

private:
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
};

} // namespace compressed_telemetry_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(compressed_telemetry_cpp::TFRelayNode)