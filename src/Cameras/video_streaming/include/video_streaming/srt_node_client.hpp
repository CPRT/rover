#pragma once

#include <gst/gst.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>

namespace video_streaming {

class SrtClientNode : public rclcpp::Node {
public:
  explicit SrtClientNode(const rclcpp::NodeOptions &options);

  ~SrtClientNode();

private:
  void gst_run_loop();

  static gboolean on_bus_message(GstBus *bus, GstMessage *msg,
                                 gpointer user_data);

  GstElement *pipeline_;
  GMainLoop *loop_;
  std::thread gst_thread_;
};

} // namespace video_streaming