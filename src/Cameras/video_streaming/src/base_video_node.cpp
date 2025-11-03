#include "base_video_node.hpp"
#include <stdexcept>

// Define the static once_flag
std::once_flag BaseVideoNode::gst_init_once_flag_;

BaseVideoNode::BaseVideoNode(const std::string name,
                             const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options) {
  // Ensure GStreamer initialized once globally (thread-safe)
  std::call_once(gst_init_once_flag_, []() { gst_init(nullptr, nullptr); });
}

BaseVideoNode::~BaseVideoNode() {
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
  }
  RCLCPP_INFO(get_logger(), "BaseVideoNode: pipeline stopped and cleaned up.");
}

GstElement *BaseVideoNode::get_element(const std::string &name) const {
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (!pipeline_)
    return nullptr;
  return gst_bin_get_by_name(GST_BIN(pipeline_), name.c_str());
}

bool BaseVideoNode::start_pipeline() {
  // Create pipeline
  if (!create_pipeline() || !pipeline_ || !GST_IS_PIPELINE(pipeline_)) {
    RCLCPP_ERROR(get_logger(), "Invalid pipeline constructed by subclass.");
    if (pipeline_) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
    throw std::runtime_error("BaseVideoNode: invalid pipeline");
  }
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) ==
        GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to set pipeline to PLAYING.");
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      throw std::runtime_error("BaseVideoNode: PLAYING failed");
    }
  }
  RCLCPP_INFO(get_logger(), "BaseVideoNode: pipeline started.");
  if (!pipeline_ || !GST_IS_PIPELINE(pipeline_)) {
    RCLCPP_ERROR(get_logger(), "Invalid pipeline constructed by subclass.");
    if (pipeline_) {
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
    }
    throw std::runtime_error("BaseVideoNode: invalid pipeline");
  }
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) ==
        GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to set pipeline to PLAYING.");
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
      pipeline_ = nullptr;
      throw std::runtime_error("BaseVideoNode: PLAYING failed");
    }
  }

  RCLCPP_INFO(get_logger(), "BaseVideoNode: pipeline started.");
  return true;
}
