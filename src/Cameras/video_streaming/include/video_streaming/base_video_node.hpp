#pragma once

#include <atomic>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <mutex>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <string>
#include <thread>

class BaseVideoNode : public rclcpp::Node {
public:
  explicit BaseVideoNode(
      const std::string name,
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~BaseVideoNode() override;

protected:
  // Convenience: look up elements by name after the pipeline is built.
  // Returns new ref; caller must unref.
  GstElement *get_element(const std::string &name) const;

  // GStreamer pipeline owned by this node.
  GstElement *pipeline_{nullptr};

  // Subclass must construct and assign `pipeline_` (GST_IS_PIPELINE(pipeline_)
  // must be true).
  virtual bool create_pipeline() = 0;

  virtual bool start_pipeline();

  virtual bool stop_pipeline();

  virtual bool pause_pipeline();

  virtual bool resume_pipeline();

  // Mutex to protect `pipeline_` access across threads
  mutable std::mutex pipeline_mutex_;
  static std::once_flag gst_init_once_flag_;

private:
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr restart_sub_;
};
