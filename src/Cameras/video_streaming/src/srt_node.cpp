#include "srt_node.hpp"
#include "interfaces/msg/srt_stats.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include <glib.h>
#include <gst/gststructure.h>
#include <gst/video/video-event.h>
#include <rclcpp_components/register_node_macro.hpp>

namespace video_streaming {

SrtNode::SrtNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("srt_node", options) {
  RCLCPP_INFO(this->get_logger(), "Initializing SrtNode...");

  // Start up
  // latency , iframe_interval
  this->declare_parameter<std::string>("srt_uri", "srt://:7001");
  this->declare_parameter<int>("latency", 100);
  this->declare_parameter<int>("iframe_interval", 0);
  this->declare_parameter<bool>("test_mode", false);

  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&SrtNode::on_parameter_change, this, std::placeholders::_1));

  bitrate_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "~/set_bitrate", 10,
      std::bind(&SrtNode::on_bitrate_received, this, std::placeholders::_1));

  iframe_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "~/trigger_iframe", 10,
      std::bind(&SrtNode::on_iframe_trigger, this, std::placeholders::_1));

  srt_stats_pub_ =
      this->create_publisher<interfaces::msg::SrtStats>("~/srt_stats", 10);

  srt_stats_timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1000),
                              std::bind(&SrtNode::publish_srt_stats, this));

  if (!start_pipeline()) {
    RCLCPP_FATAL(get_logger(), "SrtNode: failed to start pipeline.");
    throw std::runtime_error("SrtNode: pipeline start failed");
  }
  RCLCPP_INFO(get_logger(), "SrtNode constructed and pipeline started.");
}

bool SrtNode::create_pipeline() {
  std::string srt_uri = this->get_parameter("srt_uri").as_string();
  int latency_val = this->get_parameter("latency").as_int();
  int iframe_interval_val = this->get_parameter("iframe_interval").as_int();

  bool test_mode = this->get_parameter("test_mode").as_bool();

  gchar *desc;

  if (test_mode) {
    desc = g_strdup_printf(
        "videotestsrc is-live=true ! video/x-raw,width=640,height=480 ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast "
        "name=av1_enc ! "
        "rtph264pay ! queue ! "
        "srtsink name=srt_sink uri=%s latency=%d",
        srt_uri.c_str(), latency_val);
    RCLCPP_WARN(this->get_logger(), "Using MAC TEST pipeline description: %s",
                desc);

  } else {
    desc = g_strdup_printf(
        "interpipesrc listen-to=detect ! "
        "nvvidconv ! "
        "nvv4l2av1enc name=av1_enc insert-seq-hdr=true iframeinterval=%d ! "
        " av1parse ! capsfilter caps=\"video/x-av1, "
        "alignment=obu, parsed=true\" ! "
        "queue ! "
        "srtsink name=srt_sink uri=%s latency=%d",
        iframe_interval_val, srt_uri.c_str(), latency_val);
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
    GstEvent *event = gst_video_event_new_downstream_force_key_unit(
        GST_CLOCK_TIME_NONE, GST_CLOCK_TIME_NONE, GST_CLOCK_TIME_NONE, TRUE, 0);

    if (gst_element_send_event(av1_encoder_, event)) {
      RCLCPP_INFO(this->get_logger(), "I-Frame triggered (Event sent)");
    } else {
      RCLCPP_WARN(this->get_logger(), "Failed to trigger I-Frame");
    }
  }
}

void SrtNode::publish_srt_stats() {
  if (!srt_sink_)
    return;

  GstStructure *stats = nullptr;
  g_object_get(srt_sink_, "stats", &stats, NULL);

  if (stats) {
    auto msg = interfaces::msg::SrtStats();
    msg.header.stamp = this->now();

    // 1. RTT: seconds
    int rtt_ms = 0;
    if (gst_structure_get_int(stats, "rtt-ms", &rtt_ms)) {
      msg.rtt = static_cast<double>(rtt_ms) / 1000.0;
    }

    // 2. Bandwidth: Mbps -> bits/sec
    double bw_mbps = 0.0;
    if (gst_structure_get_double(stats, "bandwidth-mbps", &bw_mbps)) {
      msg.bandwidth = bw_mbps * 1e6;
    }

    // 3. Packets Sent
    int64_t val_64 = 0;
    if (gst_structure_get_int64(stats, "packets-sent", &val_64)) {
      msg.packets_sent = val_64;
    }

    // 4. Packets Lost
    int val_int = 0;
    if (gst_structure_get_int(stats, "packets-lost", &val_int) ||
        gst_structure_get_int(stats, "pkt-snd-loss-total", &val_int)) {
      msg.packets_lost = val_int;
    }

    // 5. Packets Retransmitted
    int ret_int = 0;
    if (gst_structure_get_int(stats, "packets-retransmitted", &ret_int) ||
        gst_structure_get_int(stats, "pkt-ret", &ret_int)) {
      msg.packets_retransmitted = ret_int;
    }

    srt_stats_pub_->publish(msg);
    gst_structure_free(stats); // Free the structure after use
  }
}

} // namespace video_streaming

RCLCPP_COMPONENTS_REGISTER_NODE(video_streaming::SrtNode)
