#pragma once

#include "base_video_node.hpp"

#include <gst/gst.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>

namespace video_streaming {

class RtpNode : public BaseVideoNode {
public:
  explicit RtpNode(const rclcpp::NodeOptions &options);
  ~RtpNode() override;

protected:
  bool create_pipeline() override;

private:
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);

  void set_bitrate(int32_t bitrate);
  void set_resolution(int width, int height);
  void on_iframe_trigger(const std_msgs::msg::Empty::SharedPtr msg);
  std::string get_pipeline_description();

  int target_framerate_;
  int bitrate_;
  int fec_;
  int dest_port_;
  int width_;
  int height_;

  std::string dest_ip_;

  GstElement *h265_encoder_{nullptr};

  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bitrate_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr iframe_sub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr restart_pub_;

  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace video_streaming