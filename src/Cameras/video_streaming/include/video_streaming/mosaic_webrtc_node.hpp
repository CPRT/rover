#pragma once
#include "base_video_node.hpp"

namespace video_streaming {

class MosaicWebRtcNode : public BaseVideoNode {
public:
  explicit MosaicWebRtcNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~MosaicWebRtcNode() override = default;

protected:
  bool create_pipeline() override;
};

} // namespace video_streaming
