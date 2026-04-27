#pragma once

#include "base_video_node.hpp"

#include <geometry_msgs/msg/vector3.hpp>
#include <gst/gst.h>
#include <gstnvdsmeta.h>
#include <interfaces/msg/rtp_stats.hpp>
#include <nvdsmeta.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>

namespace video_streaming {

class RtpClientNode : public BaseVideoNode {
public:
  explicit RtpClientNode(const rclcpp::NodeOptions &options);

protected:
  bool create_pipeline() override;

private:
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);
  std::string get_pipeline_description();
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  void rtp_stats_cb();
  uint16_t dest_port_;
  int latency_ms_;
  rclcpp::TimerBase::SharedPtr rtp_stats_timer_;
  rclcpp::Publisher<interfaces::msg::RtpStats>::SharedPtr rtp_stats_pub_;
  void draw_circle(NvDsBatchMeta *batch_meta, NvDsFrameMeta *frame_meta);
  int circle_x_;
  int circle_y_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr dot_sub_;
};

} // namespace video_streaming