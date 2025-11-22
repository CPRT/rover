#include "detect_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

DetectNode::DetectNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("detect_node", options) {
  RCLCPP_INFO(this->get_logger(), "DetectNode constructed.");
  start_pipeline();
}

bool DetectNode::create_pipeline() {
  // Currently just a placeholder pipeline that receives video via interpipesrc
  const char *desc =
      "interpipesrc listen-to=input is-live=true allow-renegotiation=true ! "
      "identity silent=true ! "
      "interpipesink name=detect";

  GError *err = nullptr;
  GstElement *p = gst_parse_launch(desc, &err);
  if (err || !p) {
    RCLCPP_ERROR(this->get_logger(), "gst_parse_launch failed: %s",
                 err ? err->message : "unknown");
    if (err)
      g_error_free(err);
    return false;
  }
  if (!GST_IS_PIPELINE(p)) {
    RCLCPP_ERROR(this->get_logger(), "Parsed element is not a pipeline.");
    gst_object_unref(p);
    return false;
  }

  pipeline_ = p;
  return true;
}

RCLCPP_COMPONENTS_REGISTER_NODE(DetectNode)
