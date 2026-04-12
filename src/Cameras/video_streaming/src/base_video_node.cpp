#include "base_video_node.hpp"
#include <stdexcept>

// Define the static once_flag
std::once_flag BaseVideoNode::gst_init_once_flag_;

BaseVideoNode::BaseVideoNode(const std::string name,
                             const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options) {
  // Ensure GStreamer initialized once globally (thread-safe)
  std::call_once(gst_init_once_flag_, []() { gst_init(nullptr, nullptr); });
  restart_sub_ = create_subscription<std_msgs::msg::Empty>(
      "/all_video/restart_pipeline", 10,
      [this](const std::shared_ptr<std_msgs::msg::Empty> msg) {
        try {
          if (!stop_pipeline()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to stop pipeline during restart.");
            return;
          }
          if (!start_pipeline()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to start pipeline during restart.");
            return;
          }
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(),
                       "Exception during pipeline restart: %s", e.what());
        }
      });
  graph_dot_srv_ = create_service<interfaces::srv::GraphDot>(
      "~/graph_dot",
      [this](const std::shared_ptr<interfaces::srv::GraphDot::Request> request,
             std::shared_ptr<interfaces::srv::GraphDot::Response> response) {
        std::lock_guard<std::mutex> lock(pipeline_mutex_);
        if (!pipeline_) {
          RCLCPP_ERROR(this->get_logger(),
                       "Cannot generate graph dot: pipeline is null.");
          response->success = false;
          return;
        }
        gchar *dot_data = gst_debug_bin_to_dot_data(GST_BIN(pipeline_),
                                                    GST_DEBUG_GRAPH_SHOW_ALL);
        if (!dot_data) {
          RCLCPP_ERROR(this->get_logger(),
                       "Cannot generate graph dot: gst_debug_bin_to_dot_data "
                       "returned null.");
          response->graph = "";
          response->success = false;
          return;
        }
        response->graph = dot_data;
        g_free(dot_data);
        response->success = true;
      });
}

BaseVideoNode::~BaseVideoNode() {
  stop_pipeline();
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
  return true;
}

bool BaseVideoNode::stop_pipeline() {
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (pipeline_) {
    if (gst_element_set_state(pipeline_, GST_STATE_NULL) ==
        GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to set pipeline to NULL.");
      return false;
    }
    gst_object_unref(pipeline_);
    pipeline_ = nullptr;
    RCLCPP_INFO(get_logger(), "BaseVideoNode: pipeline stopped.");
  }
  return true;
}

bool BaseVideoNode::pause_pipeline() {
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (pipeline_) {
    if (gst_element_set_state(pipeline_, GST_STATE_PAUSED) ==
        GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to set pipeline to PAUSED.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "BaseVideoNode: pipeline paused.");
  }
  return true;
}

bool BaseVideoNode::resume_pipeline() {
  std::lock_guard<std::mutex> lock(pipeline_mutex_);
  if (pipeline_) {
    if (gst_element_set_state(pipeline_, GST_STATE_PLAYING) ==
        GST_STATE_CHANGE_FAILURE) {
      RCLCPP_ERROR(get_logger(), "Failed to set pipeline to PLAYING.");
      return false;
    }
    RCLCPP_INFO(get_logger(), "BaseVideoNode: pipeline resumed.");
  }
  return true;
}

void BaseVideoNode::safe_gst_unref(GstElement *&elem) {
  if (elem) {
    gst_object_unref(elem);
    elem = nullptr;
  }
}
