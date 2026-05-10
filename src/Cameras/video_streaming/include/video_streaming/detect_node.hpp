#pragma once
#include "base_video_node.hpp"
#include <gst/app/gstappsink.h>
#include <interfaces/msg/object_detected.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

class DetectNode : public BaseVideoNode {
public:
  explicit DetectNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~DetectNode() override = default;
  void publish_object_detected(int32_t class_id, float confidence, int32_t xmin,
                               int32_t ymin, int32_t xmax, int32_t ymax);

protected:
  enum class DetectionType { WATER_BOTTLE, MALLET, ROCKPICK, ARUCO, NONE };
  bool create_pipeline() override;
  bool start_pipeline() override;

private:
  DetectionType detection_type_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr marker_pub_;
  rclcpp::Publisher<interfaces::msg::ObjectDetected>::SharedPtr
      object_detected_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr active_camera_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr arm_cam_pub_;
  std::string active_camera_{"Drive"};
  void active_camera_callback(const std_msgs::msg::String::SharedPtr msg);
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);
  DetectionType string_to_detection_type(const std::string &type_str);
  std::string detection_type_to_string() const;
  static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data);
};
