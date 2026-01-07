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

  // Set last trigger time to 1 hour ago so the first error triggers immediately
  backoff_state_.last_trigger_time =
      std::chrono::steady_clock::now() - std::chrono::hours(1);
  backoff_state_.last_loss_time = std::chrono::steady_clock::now();
  backoff_state_.last_total_dropped_pkts = 0;
  backoff_state_.current_delay_ms = INITIAL_BACKOFF_MS;

  this->declare_parameter<std::string>("srt_uri", "srt://:7001");
  this->declare_parameter<int>("latency", 100);
  this->declare_parameter<int>("iframe_interval", 0);
  this->declare_parameter<bool>("test_mode", false);
  this->declare_parameter<double>("stats_frequency", 1.0);
  this->declare_parameter<int>("target_framerate", 30);
  this->declare_parameter<int>("backoff_max_delay_ms", 5000);
  this->declare_parameter<int>("backoff_reset_ms", 10000);

  backoff_state_.max_delay_ms =
      this->get_parameter("backoff_max_delay_ms").as_int();
  backoff_state_.reset_ms = this->get_parameter("backoff_reset_ms").as_int();

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

  double stats_freq = this->get_parameter("stats_frequency").as_double();
  if (stats_freq <= 0.0) {
    stats_freq = 1.0;
    RCLCPP_WARN(this->get_logger(),
                "Invalid stats_frequency (%.2f). Defaulting to 1.0 Hz",
                stats_freq);
  }
  auto timer_period = std::chrono::duration<double>(1.0 / stats_freq);

  srt_stats_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(timer_period),
      std::bind(&SrtNode::publish_srt_stats, this));

  if (!start_pipeline()) {
    RCLCPP_FATAL(get_logger(), "SrtNode: failed to start pipeline.");
    throw std::runtime_error("SrtNode: pipeline start failed");
  }
  RCLCPP_INFO(get_logger(), "SrtNode constructed and pipeline started.");
}

SrtNode::~SrtNode() {
  BaseVideoNode::safe_gst_unref(framerate_caps_);
  BaseVideoNode::safe_gst_unref(av1_encoder_);
  BaseVideoNode::safe_gst_unref(srt_sink_);
}

bool SrtNode::create_pipeline() {
  std::string srt_uri = this->get_parameter("srt_uri").as_string();
  int latency_val = this->get_parameter("latency").as_int();
  int iframe_interval_val = this->get_parameter("iframe_interval").as_int();

  bool test_mode = this->get_parameter("test_mode").as_bool();

  int framerate = this->get_parameter("target_framerate").as_int();

  gchar *desc;

  if (test_mode) {
    desc = g_strdup_printf(
        "videotestsrc is-live=true ! "
        "videorate ! "
        "capsfilter name=framerate_caps "
        "caps=video/x-raw,framerate=%d/1 ! "
        "videoconvert ! "
        "x264enc tune=zerolatency bitrate=2000 speed-preset=ultrafast "
        "name=av1_enc ! "
        "rtph264pay ! queue ! "
        "srtsink name=srt_sink uri=%s latency=%d",
        framerate, srt_uri.c_str(), latency_val);

    RCLCPP_WARN(this->get_logger(), "Using MAC TEST pipeline description: %s",
                desc);

  } else {
    desc = g_strdup_printf(

        "interpipesrc listen-to=detect is-live=true ! "
        "videorate ! "
        "capsfilter name=framerate_caps "
        "caps=video/x-raw,framerate=%d/1 ! "
        "nvvidconv ! "
        "nvv4l2av1enc name=av1_enc insert-seq-hdr=true iframeinterval=%d ! "
        " av1parse ! capsfilter caps=\"video/x-av1, "
        "alignment=obu, parsed=true\" ! "
        "queue ! "
        "srtsink name=srt_sink uri=%s latency=%d sync=false",
        framerate, iframe_interval_val, srt_uri.c_str(), latency_val);
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
  framerate_caps_ = get_element("framerate_caps");

  if (!av1_encoder_ || !srt_sink_ || !framerate_caps_) {
    RCLCPP_ERROR(get_logger(), "Failed to get elements by name");
    return false;
  }

  RCLCPP_INFO(this->get_logger(),
              "Pipeline parsed and elements retrieved successfully.");
  return true;
}
void SrtNode::change_framerate(int new_fps) {
  if (!pipeline_) {
    RCLCPP_WARN(this->get_logger(),
                "Pipeline not initialized, cannot change framerate.");
    return;
  }

  if (!framerate_caps_) {
    RCLCPP_WARN(this->get_logger(),
                "framerate_caps_ is null! Check create_pipeline.");
    return;
  }

  GstCaps *caps = gst_caps_new_simple("video/x-raw", "framerate",
                                      GST_TYPE_FRACTION, new_fps, 1, NULL);

  g_object_set(framerate_caps_, "caps", caps, NULL);
  gst_caps_unref(caps);

  RCLCPP_INFO(this->get_logger(), "Framerate changed to %d fps", new_fps);
}

rcl_interfaces::msg::SetParametersResult
SrtNode::on_parameter_change(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &param : parameters) {
    const std::string &name = param.get_name();

    if (name == "latency") {
      if (srt_sink_) {
        int val = param.as_int();
        g_object_set(srt_sink_, "latency", val, NULL);
        RCLCPP_INFO(this->get_logger(), "Param Update: Latency set to %d", val);
      }
      continue;
    }

    if (name == "iframe_interval") {
      if (av1_encoder_) {
        int val = param.as_int();
        g_object_set(av1_encoder_, "iframeinterval", val, NULL);
        RCLCPP_INFO(this->get_logger(),
                    "Param Update: IFrame Interval set to %d", val);
      }
      continue;
    }

    if (name == "target_framerate") {
      int val = param.as_int();
      if (val > 0 && val <= 60) {
        change_framerate(val);
      } else {
        result.successful = false;
        result.reason = "Framerate out of range";
        RCLCPP_WARN(this->get_logger(), "Ignored invalid framerate: %d", val);
      }
      continue;
    }

    if (name == "backoff_max_delay_ms") {
      int val = param.as_int();
      if (val > 0) {
        backoff_state_.max_delay_ms = val;
        RCLCPP_INFO(this->get_logger(),
                    "Param Update: Backoff Max Delay set to %d ms", val);
      }
      continue;
    }

    if (name == "backoff_reset_ms") {
      int val = param.as_int();
      if (val > 0) {
        backoff_state_.reset_ms = val;
        RCLCPP_INFO(this->get_logger(),
                    "Param Update: Backoff Reset Time set to %d ms", val);
      }
      continue;
    }
  }

  return result;
}

void SrtNode::on_bitrate_received(const std_msgs::msg::Int32::SharedPtr msg) {
  if (av1_encoder_) {
    pause_pipeline();
    g_object_set(av1_encoder_, "bitrate", msg->data, NULL);
    resume_pipeline();
    RCLCPP_DEBUG(this->get_logger(), "Bitrate set to %d", msg->data);
  }
}

void SrtNode::on_iframe_trigger(const std_msgs::msg::Empty::SharedPtr msg) {
  (void)msg;

  if (srt_sink_) {

    GstEvent *event = gst_video_event_new_upstream_force_key_unit(
        GST_CLOCK_TIME_NONE, TRUE, 0);

    if (gst_element_send_event(srt_sink_, event)) {
      RCLCPP_INFO(this->get_logger(),
                  "Manual I-Frame triggered (Upstream Event sent)");
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Failed to trigger Manual I-Frame (Event rejected)");
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "Cannot trigger I-Frame: SRT Sink is null");
  }
}

void SrtNode::check_packet_loss_and_trigger(int64_t current_total_dropped) {
  auto now = std::chrono::steady_clock::now();

  if (current_total_dropped <= backoff_state_.last_total_dropped_pkts) {
    auto time_since_loss =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now - backoff_state_.last_loss_time)
            .count();

    if (time_since_loss > backoff_state_.reset_ms &&
        backoff_state_.current_delay_ms != INITIAL_BACKOFF_MS) {
      RCLCPP_DEBUG(this->get_logger(),
                   "SRT connection stable. Resetting backoff.");
    }
    backoff_state_.current_delay_ms = INITIAL_BACKOFF_MS;
    return;
  }

  backoff_state_.last_total_dropped_pkts = current_total_dropped;
  backoff_state_.last_loss_time = now;

  auto time_since_trigger =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          now - backoff_state_.last_trigger_time)
          .count();

  if (time_since_trigger < backoff_state_.current_delay_ms) {
    RCLCPP_DEBUG(this->get_logger(),
                 "SRT Packet Drop Detected (%ld total). Suppressing I-Frame "
                 "request (Backoff in effect: %d ms)",
                 time_since_trigger, backoff_state_.current_delay_ms);
    return;
  }

  RCLCPP_WARN(this->get_logger(),
              "SRT Packet Drop Detected (%ld total). Requesting I-Frame! (Next "
              "backoff: %d ms)",
              current_total_dropped, backoff_state_.current_delay_ms * 2);

  GstEvent *event =
      gst_video_event_new_upstream_force_key_unit(GST_CLOCK_TIME_NONE, TRUE, 0);
  if (srt_sink_) {
    gst_element_send_event(srt_sink_, event);
  }

  backoff_state_.last_trigger_time = now;
  backoff_state_.current_delay_ms = std::min(
      backoff_state_.current_delay_ms * 2, backoff_state_.max_delay_ms);
}

void SrtNode::publish_srt_stats() {
  if (!srt_sink_) {
    return;
  }
  GstStructure *stats = nullptr;
  g_object_get(srt_sink_, "stats", &stats, NULL);

  if (!stats) {
    return;
  }
  auto msg = interfaces::msg::SrtStats();
  msg.header.stamp = this->now();

  GstStructure *target_stats = stats;
  const GValue *callers_val = gst_structure_get_value(stats, "callers");

  if (callers_val) {
    GValueArray *arr = (GValueArray *)g_value_get_boxed(callers_val);
    if (arr && arr->n_values > 0) {
      GValue *first_val = g_value_array_get_nth(arr, 0);
      target_stats = (GstStructure *)g_value_get_boxed(first_val);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *(this->get_clock()), 10,
                           "No SRT caller stats available currently - (Msg is "
                           "throttled to 10s)");
      gst_structure_free(stats);
      return;
    }
  }

  // 1. RTT: milliseconds
  double rtt_ms = 0.0;
  if (gst_structure_get_double(target_stats, "rtt-ms", &rtt_ms)) {
    msg.rtt = rtt_ms;
  }

  // 2. Bandwidth: Mbps -> bits/sec
  double bw_mbps = 0.0;
  if (gst_structure_get_double(target_stats, "bandwidth-mbps", &bw_mbps)) {
    msg.bandwidth = bw_mbps * 1e6;
  }

  // 3. Packets Sent
  int64_t val_64 = 0;
  if (gst_structure_get_int64(target_stats, "packets-sent", &val_64)) {
    msg.packets_sent = val_64;
  }

  // 4. Packets Lost
  int val_int = 0;
  if (gst_structure_get_int(target_stats, "packets-sent-lost", &val_int)) {
    msg.packets_lost = val_int;
  }

  // 5. Packets Retransmitted
  int ret_int = 0;
  if (gst_structure_get_int(target_stats, "packets-retransmitted", &ret_int)) {
    msg.packets_retransmitted = ret_int;
  }

  int64_t pkt_drop_total = 0;

  if (gst_structure_get_int64(target_stats, "pkt-drop-total",
                              &pkt_drop_total)) {
    msg.packet_drop_total = pkt_drop_total;
    check_packet_loss_and_trigger(pkt_drop_total);
  }

  srt_stats_pub_->publish(msg);
  gst_structure_free(stats);
}

RCLCPP_COMPONENTS_REGISTER_NODE(video_streaming::SrtNode)
} // namespace video_streaming
