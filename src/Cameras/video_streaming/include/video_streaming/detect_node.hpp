#pragma once
#include "base_video_node.hpp"
#include <atomic>
#include <interfaces/msg/object_detected.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/string.hpp>

class DetectNode : public BaseVideoNode {
public:
  explicit DetectNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~DetectNode() override = default;
  void publish_object_detected(int32_t class_id, float confidence, int32_t xmin,
                               int32_t ymin, int32_t xmax, int32_t ymax);

  void publish_morse_character(guint char_code);
  void publish_morse_text(const std::string &text);
  void sync_morse_calibrate_state(bool calibrating);
  void sync_morse_roi_state(int roi_x, int roi_y);

protected:
  enum class DetectionType {
    WATER_BOTTLE,
    MALLET,
    ROCKPICK,
    ARUCO,
    MORSE,
    NONE
  };
  bool create_pipeline() override;
  bool start_pipeline() override;

private:
  void declare_morse_parameters();
  void apply_morse_element_properties(GstElement *morse);
  void on_parameters_set(const std::vector<rclcpp::Parameter> &parameters);

  DetectionType detection_type_;
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rclcpp::Node::PostSetParametersCallbackHandle::SharedPtr
      post_set_param_callback_handle_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr marker_pub_;
  rclcpp::Publisher<interfaces::msg::ObjectDetected>::SharedPtr
      object_detected_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr morse_character_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr morse_text_pub_;
  std::atomic<bool> applying_morse_plugin_sync_{false};

  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);
  DetectionType string_to_detection_type(const std::string &type_str);
  std::string detection_type_to_string() const;
};
