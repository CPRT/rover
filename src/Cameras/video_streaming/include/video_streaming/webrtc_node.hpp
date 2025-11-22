#pragma once
#include "base_video_node.hpp"

class WebRtcNode : public BaseVideoNode {
public:
  explicit WebRtcNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~WebRtcNode() override = default;

protected:
  bool create_pipeline() override;
};
