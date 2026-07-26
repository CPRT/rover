#pragma once
#include "base_video_node.hpp"
#include <atomic>
#include <std_msgs/msg/string.hpp>

class MorseLedNode : public BaseVideoNode {
public:
  explicit MorseLedNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MorseLedNode() override = default;

  void publish_character(guint char_code);
  void publish_decoded_text(const std::string &text);

  void sync_calibrate_state(bool calibrating);
  void sync_roi_state(int roi_x, int roi_y);

protected:
  bool create_pipeline() override;
  bool start_pipeline() override;

private:
  void declare_parameters();
  void apply_element_properties(GstElement *morse);
  void on_parameters_set(const std::vector<rclcpp::Parameter> &parameters);

  rclcpp::Node::PostSetParametersCallbackHandle::SharedPtr
      post_set_param_callback_handle_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr character_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr text_pub_;

  std::atomic<bool> applying_plugin_sync_{false};
};
