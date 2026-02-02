#include "srt_node_client.hpp"
#include <rclcpp_components/register_node_macro.hpp>

namespace video_streaming {

SrtClientNode::SrtClientNode(const rclcpp::NodeOptions &options)
    : Node("srt_client_node", options), pipeline_(nullptr), loop_(nullptr) {
  if (!gst_is_initialized()) {
    gst_init(nullptr, nullptr);
  }

  this->declare_parameter<std::string>("srt_uri",
                                       "srt://192.168.0.55:7001?mode=caller");
  std::string uri = this->get_parameter("srt_uri").as_string();

  RCLCPP_INFO(this->get_logger(), "SRT URI: %s", uri.c_str());

  std::string pipeline_str = "srtsrc uri=\"" + uri +
                             "\" latency=0 ! "
                             "tsdemux latency=0 ! "
                             "h265parse ! "
                            //  "parsebin ! test "
                             "nvv4l2decoder ! "
                             "nvvidconv ! "
                             "queue !"
                             "nveglglessink sync=false";

  RCLCPP_INFO(this->get_logger(), "Pipeline: %s", pipeline_str.c_str());

  GError *error = nullptr;
  pipeline_ = gst_parse_launch(pipeline_str.c_str(), &error);

  if (error) {
    RCLCPP_ERROR(this->get_logger(), "Failed to parse pipeline: %s",
                 error->message);
    g_error_free(error);
    return;
  }

  GstBus *bus = gst_element_get_bus(pipeline_);
  gst_bus_add_watch(bus, (GstBusFunc)SrtClientNode::on_bus_message, this);
  gst_object_unref(bus);

  loop_ = g_main_loop_new(nullptr, FALSE);

  gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  RCLCPP_INFO(this->get_logger(), "Pipeline started. Waiting for stream...");

  gst_thread_ = std::thread(&SrtClientNode::gst_run_loop, this);
}

SrtClientNode::~SrtClientNode() {
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  if (loop_ && g_main_loop_is_running(loop_)) {
    g_main_loop_quit(loop_);
  }

  if (gst_thread_.joinable()) {
    gst_thread_.join();
  }

  if (pipeline_) {
    gst_element_set_state(pipeline_, GST_STATE_NULL);
    gst_object_unref(pipeline_);
  }
  if (loop_) {
    g_main_loop_unref(loop_);
  }
}

void SrtClientNode::gst_run_loop() { g_main_loop_run(loop_); }

gboolean SrtClientNode::on_bus_message(GstBus *bus, GstMessage *msg,
                                       gpointer user_data) {
  SrtClientNode *node = static_cast<SrtClientNode *>(user_data);

  switch (GST_MESSAGE_TYPE(msg)) {
  case GST_MESSAGE_EOS:
    RCLCPP_INFO(node->get_logger(), "End of stream.");
    g_main_loop_quit(node->loop_);
    break;

  case GST_MESSAGE_ERROR: {
    GError *err = nullptr;
    gchar *dbg_info = nullptr;

    gst_message_parse_error(msg, &err, &dbg_info);
    RCLCPP_ERROR(node->get_logger(), "ERROR from element %s: %s",
                 GST_OBJECT_NAME(msg->src), err->message);
    RCLCPP_ERROR(node->get_logger(), "Debugging info: %s",
                 (dbg_info) ? dbg_info : "none");

    g_error_free(err);
    g_free(dbg_info);

    g_main_loop_quit(node->loop_);
    break;
  }

  case GST_MESSAGE_STATE_CHANGED: {
    GstState old_state, new_state, pending_state;
    gst_message_parse_state_changed(msg, &old_state, &new_state,
                                    &pending_state);
    if (GST_MESSAGE_SRC(msg) == GST_OBJECT(node->pipeline_)) {
      RCLCPP_INFO(node->get_logger(), "Pipeline state changed from %s to %s",
                  gst_element_state_get_name(old_state),
                  gst_element_state_get_name(new_state));
    }
    break;
  }

  default:
    break;
  }

  return TRUE;
}

} // namespace video_streaming

RCLCPP_COMPONENTS_REGISTER_NODE(video_streaming::SrtClientNode)