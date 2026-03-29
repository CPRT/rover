#include "rtp_client_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
namespace video_streaming {
RtpClientNode::RtpClientNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("rtp_client_node", options) {
  RCLCPP_INFO(this->get_logger(), "Initializing RtpClientNode...");

  this->declare_parameter<int>("port", 7001);
  this->declare_parameter<int>("latency_ms", 200);
  this->declare_parameter<double>("stats_frequency", 1.0);
  dest_port_ = this->get_parameter("port").as_int();
  latency_ms_ = this->get_parameter("latency_ms").as_int();
  param_callback_handle_ = this->add_on_set_parameters_callback(std::bind(
      &RtpClientNode::on_parameter_change, this, std::placeholders::_1));
  if (!start_pipeline()) {
    RCLCPP_FATAL(get_logger(), "RtpClientNode: failed to start pipeline.");
    throw std::runtime_error("RtpClientNode: pipeline start failed");
  }
  double stats_freq = this->get_parameter("stats_frequency").as_double();
  if (stats_freq <= 0.0) {
    stats_freq = 1.0;
    RCLCPP_WARN(this->get_logger(),
                "Invalid stats_frequency (%.2f). Defaulting to 1.0 Hz",
                stats_freq);
  }
  auto timer_period = std::chrono::duration<double>(1.0 / stats_freq);
  rtp_stats_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
      std::bind(&RtpClientNode::rtp_stats_cb, this));
  rtp_stats_pub_ =
      this->create_publisher<interfaces::msg::RtpStats>("~/rtp_stats", 10);
  RCLCPP_INFO(get_logger(), "RtpClientNode constructed and pipeline started.");
}

std::string RtpClientNode::get_pipeline_description() {
  std::stringstream desc;
  desc << "udpsrc port=" << dest_port_ << " caps="
       << "\"application/x-rtp,media=video,encoding-name=H265,payload=96, "
       << "clock-rate=90000\" ! rtpjitterbuffer name=rtp_buf mode=4 latency="
       << latency_ms_
       << " drop-on-latency=true ! rtpulpfecdec name=fec_dec ! rtph265depay ! "
          "h265parse ! "
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

void RtpClientNode::rtp_stats_cb() {

  // Get data from jitter buffer
  GstElement *jitterbuffer = get_element("rtp_buf");
  if (!jitterbuffer) {
    RCLCPP_WARN(this->get_logger(), "Could not get jitterbuffer");
    return;
  }
  GstStructure *stats_struct = nullptr;
  g_object_get(jitterbuffer, "stats", &stats_struct, NULL);
  g_object_unref(jitterbuffer);
  if (!stats_struct) {
    RCLCPP_WARN(this->get_logger(), "Failed to get stats");
    return;
  }
  guint64 num_lost = 0;
  guint64 num_pushed = 0;
  guint64 num_late = 0;
  guint64 num_duplicates = 0;
  guint64 avg_jitter = 0;
  gst_structure_get_uint64(stats_struct, "num-lost", &num_lost);
  gst_structure_get_uint64(stats_struct, "num-pushed", &num_pushed);
  gst_structure_get_uint64(stats_struct, "num-late", &num_late);
  gst_structure_get_uint64(stats_struct, "num-duplicates", &num_duplicates);
  gst_structure_get_uint64(stats_struct, "avg-jitter", &avg_jitter);
  gst_structure_free(stats_struct);

  // Get data from fec decoder
  GstElement *fec_dec = get_element("fec_dec");
  if (!fec_dec) {
    RCLCPP_WARN(this->get_logger(), "Could not get fec_dec");
    return;
  }
  guint recovered = 0;
  guint unrecovered = 0;
  g_object_get(fec_dec, "recovered", &recovered, "unrecovered", &unrecovered,
               NULL);
  g_object_unref(fec_dec);

  auto msg = interfaces::msg::RtpStats();
  msg.header.stamp = this->now();
  msg.num_pushed = num_pushed;
  msg.num_lost = num_lost;
  msg.num_late = num_late;
  msg.num_duplicates = num_duplicates;
  msg.avg_jitter = avg_jitter;
  msg.recovered = recovered;
  msg.unrecovered = unrecovered;
  rtp_stats_pub_->publish(msg);
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