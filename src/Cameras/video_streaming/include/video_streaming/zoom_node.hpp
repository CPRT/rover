#pragma once
#include "base_video_node.hpp"
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>

namespace video_streaming {

class ZoomNode : public BaseVideoNode {
public:
  explicit ZoomNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~ZoomNode() override = default;

protected:
  bool create_pipeline() override;

private:
  void zoom_cb(const std_msgs::msg::Float32::SharedPtr msg);
  void pan_cb(const std_msgs::msg::Int8::SharedPtr msg);
  void tilt_cb(const std_msgs::msg::Int8::SharedPtr msg);
  void update();

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr zoom_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr pan_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr tilt_sub_;

  int8_t pan_{50};
  int8_t tilt_{50};
  float zoom_{1.0f};
};

} // namespace video_streaming