#include "srt_node.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include <glib.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace video_streaming {

SrtNode::SrtNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("srt_node", options) {
  RCLCPP_INFO(this->get_logger(), "Initializing SrtNode...");

  // Start up
  // latency , iframe_interval
  this->declare_parameter<std::string>("srt_uri", "srt://127.0.0.1:7001");
  this->declare_parameter<int>("latency", 100);
  this->declare_parameter<int>("iframe_interval", 0);

  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SrtNode::on_parameter_change, this, std::placeholders::_1));

  bitrate_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "~/set_bitrate", 10,
      std::bind(&SrtNode::on_bitrate_received, this, std::placeholders::_1));

  iframe_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "~/trigger_iframe", 10,
      std::bind(&SrtNode::on_iframe_trigger, this, std::placeholders::_1));

  if (!start_pipeline()) {
    RCLCPP_FATAL(get_logger(), "SrtNode: failed to start pipeline.");
    throw std::runtime_error("SrtNode: pipeline start failed");
  }
  RCLCPP_INFO(get_logger(), "SrtNode constructed and pipeline started.");
}

bool SrtNode::create_pipeline() {
  std::string srt_uri = this->get_parameter("srt_uri").as_string();
  int latency_val = this->get_parameter("latency").as_int();

  bool test_mode = false;
  try {
    test_mode = this->get_parameter("test_mode").as_bool();
  } catch (...) {
    test_mode = this->declare_parameter<bool>("test_mode", false);
  }

  gchar *desc;

  if (test_mode) {
    desc = g_strdup_printf(
        "videotestsrc is-live=true ! video/x-raw,width=640,height=480 ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast "
        "name=av1_enc ! "
        "rtph264pay ! queue ! "
        "srtsink name=srt_sink uri=%s latency=%d mode=1",
        srt_uri.c_str(), latency_val);
    RCLCPP_WARN(this->get_logger(), "Using MAC TEST pipeline description: %s",
                desc);

  } else {
    desc = g_strdup_printf("interpipesrc listen-to=detect ! "
                           "nvvidconv ! "
                           "nvv4l2av1enc name=av1_enc ! "
                           "rtpav1pay ! queue ! "
                           "srtsink name=srt_sink uri=%s latency=%d mode=1",
                           srt_uri.c_str(), latency_val);
    RCLCPP_INFO(this->get_logger(), "Using PRODUCTION pipeline description: %s",
                desc);
  }
  RCLCPP_INFO(this->get_logger(), "Pipeline description: %s", desc);

  GError *err = nullptr;
  GstElement *p = gst_parse_launch(desc, &err);
  g_free(desc);

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

  av1_encoder_ = this->get_element("av1_enc");
  srt_sink_ = this->get_element("srt_sink");

  if (!av1_encoder_ || !srt_sink_) {
    RCLCPP_ERROR(get_logger(), "Failed to get elements by name");
    return false;
  }

  RCLCPP_INFO(this->get_logger(),
              "Pipeline parsed and elements retrieved successfully.");
  return true;
}

rcl_interfaces::msg::SetParametersResult
SrtNode::on_parameter_change(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &param : parameters) {
    if (param.get_name() == "latency" && srt_sink_) {
      int val = param.as_int();
      g_object_set(srt_sink_, "latency", val, NULL);
      RCLCPP_INFO(this->get_logger(), "Param Update: Latency set to %d", val);
    } else if (param.get_name() == "iframe_interval" && av1_encoder_) {
      int val = param.as_int();

      g_object_set(av1_encoder_, "iframeinterval", val, NULL);
      RCLCPP_INFO(this->get_logger(), "Param Update: IFrame Interval set to %d",
                  val);
    }
  }
  return result;
}

void SrtNode::on_bitrate_received(const std_msgs::msg::Int32::SharedPtr msg) {
  if (av1_encoder_) {
    g_object_set(av1_encoder_, "bitrate", msg->data, NULL);
    RCLCPP_INFO(this->get_logger(), "Bitrate set to %d", msg->data);
  }
}

void SrtNode::on_iframe_trigger(const std_msgs::msg::Empty::SharedPtr msg) {
  (void)msg;
  if (av1_encoder_) {
    g_signal_emit_by_name(av1_encoder_, "force-key-unit", NULL);
    RCLCPP_INFO(this->get_logger(), "I-Frame triggered");
  }
}

} // namespace video_streaming

RCLCPP_COMPONENTS_REGISTER_NODE(video_streaming::SrtNode)