#include "srt_node.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include <glib.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace video_streaming
{

SrtNode::SrtNode(const rclcpp::NodeOptions & options) : BaseVideoNode("srt_node", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing SrtNode...");

  // Start up
  // latency , iframe_interval
  this->declare_parameter<std::string>("srt_uri", "srt://127.0.0.1:12345");
  this->declare_parameter<int>("latency", 100);
  this->declare_parameter<int>("iframe_interval", 0);

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&SrtNode::on_parameter_change, this, std::placeholders::_1));

  bitrate_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "~/set_bitrate", 10, std::bind(&SrtNode::on_bitrate_received, this, std::placeholders::_1));

  iframe_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "~/trigger_iframe", 10, std::bind(&SrtNode::on_iframe_trigger, this, std::placeholders::_1));

  if (!start_pipeline()) {
    RCLCPP_FATAL(get_logger(), "SrtNode: failed to start pipeline.");
    throw std::runtime_error("SrtNode: pipeline start failed");
  }
  RCLCPP_INFO(get_logger(), "SrtNode constructed and pipeline started.");
}

bool SrtNode::create_pipeline()
{
  std::string srt_uri = this->get_parameter("srt_uri").as_string();
  int latency_val = this->get_parameter("latency").as_int();

  // ✅ PRODUCTION MODE (NVIDIA Jetson)
  // Added comma correctly below
  gchar * desc = g_strdup_printf(
    "interpipesrc listen-to=detect ! "
    "nvvidconv ! "
    "nvv4l2av1enc name=av1_enc ! "
    "rtpav1pay ! queue ! "
    "srtsink name=srt_sink uri=%s latency=%d mode=1",
    srt_uri.c_str(), latency_val);

  /* // [MAC TEST MODE]
    gchar *desc = g_strdup_printf(
        "videotestsrc is-live=true ! video/x-raw,width=640,height=480 ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast name=av1_enc ! "
        "rtph264pay ! queue ! "
        "srtsink name=srt_sink uri=%s latency=%d mode=1",
        srt_uri.c_str(),
        latency_val
    );
    */

  RCLCPP_INFO(this->get_logger(), "Using PRODUCTION pipeline description: %s", desc);

  GError * err = nullptr;
  GstElement * p = gst_parse_launch(desc, &err);
  g_free(desc);

  if (err || !p) {
    RCLCPP_ERROR(get_logger(), "gst_parse_launch failed: %s", err ? err->message : "unknown");
    if (err) g_error_free(err);
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

  RCLCPP_INFO(this->get_logger(), "Pipeline parsed and elements retrieved successfully.");
  return true;
}

rcl_interfaces::msg::SetParametersResult SrtNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & param : parameters) {
    if (param.get_name() == "latency" && srt_sink_) {
      int val = param.as_int();
      g_object_set(srt_sink_, "latency", val, NULL);
      RCLCPP_INFO(this->get_logger(), "Param Update: Latency set to %d", val);
    } else if (param.get_name() == "iframe_interval" && av1_encoder_) {
      int val = param.as_int();
      // ✅ Production property name: iframeinterval
      g_object_set(av1_encoder_, "iframeinterval", val, NULL);
      RCLCPP_INFO(this->get_logger(), "Param Update: IFrame Interval set to %d", val);
    }
  }
  return result;
}

void SrtNode::on_bitrate_received(const std_msgs::msg::Int32::SharedPtr msg)
{
  if (av1_encoder_) {
    g_object_set(av1_encoder_, "bitrate", msg->data, NULL);
    RCLCPP_INFO(this->get_logger(), "Bitrate set to %d", msg->data);
  }
}

void SrtNode::on_iframe_trigger(const std_msgs::msg::Empty::SharedPtr msg)
{
  (void)msg;
  if (av1_encoder_) {
    g_signal_emit_by_name(av1_encoder_, "force-key-unit", NULL);
    RCLCPP_INFO(this->get_logger(), "I-Frame triggered");
  }
}

/* ==================================================================================
 * TODO: FUTURE FEATURE - SRT STATISTICS & SERVICE
 * * The following code implements SRT statistics publishing and a custom service.
 * * Per feedback, we are currently pausing this functionality to focus on the video
 * * pipeline and using ROS 2 Parameters for control.
 * * Once custom .msg and .srv files are added to the interfaces package,
 * * uncomment this block and the corresponding headers.
 * ==================================================================================
 */

/*
// void SrtNode::on_srt_stats(GstElement * element, GstStructure * stats, gpointer user_data)
// {
//     SrtNode* node = static_cast<SrtNode*>(user_data);
//     if (!node) return;

//     video_streaming::msg::Srtstat stats_msg;
//     if (!stats) {
//         RCLCPP_WARN(node->get_logger(), "on_srt_stats called with null GstStructure");
//         node->stats_pub_->publish(stats_msg);
//         return;
//     }

//     // Debug: print the full structure to discover actual key names & units
//     gchar *s = gst_structure_to_string(stats);
//     RCLCPP_DEBUG(node->get_logger(), "SRT stats structure: %s", s);
//     g_free(s);

//     // rtt -> Srtstat.rtt (seconds)
//     double rtt_double = 0.0;
//     gint rtt_int = 0;
//     if (gst_structure_get_double(stats, "rtt", &rtt_double)) {
//         // assume already in seconds
//         stats_msg.rtt = rtt_double;
//     } else if (gst_structure_get_int(stats, "rtt", &rtt_int)) {
//         // some sources report ms as integer — convert to seconds
//         stats_msg.rtt = static_cast<double>(rtt_int) / 1000.0;
//     }

//     // bandwidth -> Srtstat.bandwidth (bits/sec)
//     double bw_double = 0.0;
//     gint bw_int = 0;
//     if (gst_structure_get_double(stats, "bandwidth", &bw_double)) {
//         stats_msg.bandwidth = bw_double;
//     } else if (gst_structure_get_int(stats, "bandwidth", &bw_int)) {
//         stats_msg.bandwidth = static_cast<double>(bw_int);
//     }

//     // packets_sent -> Srtstat.packets_sent
//     gint packets_sent = 0;
//     if (gst_structure_get_int(stats, "packets-sent", &packets_sent) ||
//         gst_structure_get_int(stats, "packets_sent", &packets_sent)) {
//         stats_msg.packets_sent = static_cast<int64_t>(packets_sent);
//     }

//     // packets_lost -> Srtstat.packets_lost
//     gint packets_lost = 0;
//     if (gst_structure_get_int(stats, "packets-lost", &packets_lost) ||
//         gst_structure_get_int(stats, "packets_lost", &packets_lost)) {
//         stats_msg.packets_lost = static_cast<int64_t>(packets_lost);
//     }

//     // packets_retransmitted -> Srtstat.packets_retransmitted
//     gint packets_retx = 0;
//     if (gst_structure_get_int(stats, "packets-retransmitted", &packets_retx) ||
//         gst_structure_get_int(stats, "packets_retransmitted", &packets_retx)) {
//         stats_msg.packets_retransmitted = static_cast<int64_t>(packets_retx);
//     }

//     node->stats_pub_->publish(stats_msg);
// }
*/

}  // namespace video_streaming

RCLCPP_COMPONENTS_REGISTER_NODE(video_streaming::SrtNode)