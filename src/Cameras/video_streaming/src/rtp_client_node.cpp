#include "rtp_client_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
namespace video_streaming {
RtpClientNode::RtpClientNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("rtp_client_node", options) {
  RCLCPP_INFO(this->get_logger(), "Initializing RtpClientNode...");

  this->declare_parameter<int>("port", 7001);
  this->declare_parameter<int>("latency_ms", 200);
  dest_port_ = this->get_parameter("port").as_int();
  latency_ms_ = this->get_parameter("latency_ms").as_int();
  param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &RtpClientNode::on_parameter_change, this, std::placeholders::_1));
  if (!start_pipeline()) {
    RCLCPP_FATAL(get_logger(), "RtpClientNode: failed to start pipeline.");
    throw std::runtime_error("RtpClientNode: pipeline start failed");
  }
  RCLCPP_INFO(get_logger(), "RtpClientNode constructed and pipeline started.");
}

std::string RtpClientNode::get_pipeline_description() {
  std::stringstream desc;
  desc << "udpsrc port=" << dest_port_ << " caps="
       << "\"application/x-rtp,media=video,encoding-name=H265,payload=96, "
       << "clock-rate=90000\" ! rtpjitterbuffer mode=4 latency=" << latency_ms_
       << " drop-on-latency=true ! rtpulpfecdec ! rtph265depay ! h265parse ! "
       << "nvv4l2decoder ! nvvidconv ! nveglglessink sync=false";
  return desc.str();
}

bool RtpClientNode::create_pipeline() {
  std::string desc = get_pipeline_description();
  RCLCPP_INFO(this->get_logger(), "Pipeline description: %s", desc.c_str());

  GError *err = nullptr;
  GstElement *p = gst_parse_launch(desc.c_str(), &err);

  if (err || !p) {
    RCLCPP_ERROR(get_logger(), "gst_parse_launch failed: %s",
                 err ? err->message : "unknown");
    if (err) {
      g_error_free(err);
    }
    return false;
  }
  if (!GST_IS_PIPELINE(p)) {
    RCLCPP_ERROR(get_logger(), "Parsed element is not a pipeline.");
    gst_object_unref(p);
    return false;
  }
  pipeline_ = p;

  RCLCPP_INFO(this->get_logger(), "Pipeline parsed successfully.");
  return true;
}
rcl_interfaces::msg::SetParametersResult RtpClientNode::on_parameter_change(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  bool needs_restart = false;
  for (const auto &param : parameters) {
    const std::string &name = param.get_name();

    // Destination Port
    if (name == "port") {
      dest_port_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Param Update: port = %d", dest_port_);
      needs_restart = true;
      continue;
    }
    // Latency
    if (name == "latency_ms") {
      latency_ms_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Param Update: latency_ms = %d",
                  latency_ms_);
      needs_restart = true;
      continue;
    }
  }
  if (needs_restart) {
    stop_pipeline();
    start_pipeline();
  }
  return result;
}
RCLCPP_COMPONENTS_REGISTER_NODE(video_streaming::RtpClientNode);
} // namespace video_streaming