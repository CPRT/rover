#pragma once
#include "base_video_node.hpp"
#include <std_msgs/msg/string.hpp>

class MorseLedNode : public BaseVideoNode {
public:
  explicit MorseLedNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MorseLedNode() override = default;

  void publish_character(guint char_code);
  void publish_decoded_text(const std::string &text);

protected:
  bool create_pipeline() override;
  bool start_pipeline() override;

private:
  void declare_parameters();
  void apply_element_properties(GstElement *morse);
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr character_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;
};
