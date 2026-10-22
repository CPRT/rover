#include "webrtc_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

WebRtcNode::WebRtcNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("webrtc_node", options) {
  if (!start_pipeline()) {
    RCLCPP_FATAL(get_logger(), "WebRtcNode: failed to start pipeline.");
    throw std::runtime_error("WebRtcNode: pipeline start failed");
  }
  RCLCPP_INFO(get_logger(), "WebRtcNode constructed and pipeline started.");
}

bool WebRtcNode::create_pipeline() {
  const char *desc = "interpipesrc listen-to=detect ! "
                     "identity silent=true ! nvvidconv ! "
                     "webrtcsink run-signalling-server=true "
                     "video-caps=\"video/x-h265; video/x-h264\"";

  GError *err = nullptr;
  GstElement *p = gst_parse_launch(desc, &err);
  if (err || !p) {
    RCLCPP_ERROR(get_logger(), "gst_parse_launch failed: %s",
                 err ? err->message : "unknown");
    if (err)
      g_error_free(err);
    return false;
  }
  if (!GST_IS_PIPELINE(p)) {
    RCLCPP_ERROR(get_logger(), "Parsed element is not a pipeline.");
    gst_object_unref(p);
    return false;
  }

  pipeline_ = p;
  return true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(WebRtcNode)
