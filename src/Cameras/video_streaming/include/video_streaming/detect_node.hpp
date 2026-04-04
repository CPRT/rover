#pragma once
#include "base_video_node.hpp"
#include <std_msgs/msg/int32.hpp>

class DetectNode : public BaseVideoNode {
public:
  explicit DetectNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~DetectNode() override = default;

protected:
  enum class DetectionType { WATER_BOTTLE, MALLET, ROCKPICK, ARUCO, NONE };
  bool create_pipeline() override;
  bool start_pipeline() override;

private:
  DetectionType detection_type_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr marker_pub_;
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);
  DetectionType string_to_detection_type(const std::string &type_str);
};
