#pragma once
#include "base_video_node.hpp"

class DetectNode : public BaseVideoNode {
public:
  explicit DetectNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~DetectNode() override = default;

protected:
  bool create_pipeline() override;
};
