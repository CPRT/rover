#include "rtp_client_node.hpp"
#include <gstnvdsmeta.h>
#include <nvdsmeta.h>
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

  dot_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "~/dot", 2, [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        circle_x_ = static_cast<int>(msg->x);
        circle_y_ = static_cast<int>(msg->y);
      });
}

std::string RtpClientNode::get_pipeline_description() {
  std::stringstream desc;
  constexpr guint storage_tolerance_ms = 20;
  guint64 storage_sz_ns = (latency_ms_ + storage_tolerance_ms) * 1000000ULL;
  desc << "udpsrc port=" << dest_port_ << " caps="
       << "\"application/x-rtp, payload=96, clock-rate=90000\" ! "
       << "rtpstorage size-time=" << storage_sz_ns
       << " ! rtpssrcdemux ! capsfilter "
          "caps=\"application/"
          "x-rtp,media=video,encoding-name=H265,payload=96,clock-rate=90000\" "
          "! rtpjitterbuffer name=rtp_buf mode=4 latency="
       << latency_ms_
       << " ! rtph265depay ! h265parse ! nvv4l2decoder ! nvvidconv ! "
       << "capsfilter caps=\"video/x-raw(memory:NVMM),width=1920,height=1080\" "
       << " ! queue ! mux.sink_0 "
       << "nvstreammux batch-size=1 width=1920 height=1080 name=mux ! "
       << "nvdsosd name=render ! nvvidconv ! nveglglessink sync=false";

  return desc.str();
}

void RtpClientNode::draw_circle(NvDsBatchMeta *batch_meta,
                                NvDsFrameMeta *frame_meta) {
  if (circle_x_ < 0 || circle_y_ < 0) {
    return;
  }
  NvDsDisplayMeta *display_meta =
      nvds_acquire_display_meta_from_pool(batch_meta);

  if (!display_meta) {
    RCLCPP_ERROR(get_logger(), "Failed to acquire display meta.");
    return;
  }

  if (display_meta->num_circles >= MAX_ELEMENTS_IN_DISPLAY_META) {
    RCLCPP_WARN(get_logger(), "Display meta circle limit reached.");
    return;
  }

  NvOSD_CircleParams &circle_params =
      display_meta->circle_params[display_meta->num_circles];

  circle_params.xc = circle_x_;
  circle_params.yc = circle_y_;
  circle_params.radius = 10;
  circle_params.circle_color.red = 1.0;
  circle_params.circle_color.green = 0.0;
  circle_params.circle_color.blue = 1.0;
  circle_params.circle_color.alpha = 1.0;

  circle_params.bg_color.red = 0.0;
  circle_params.bg_color.green = 1.0;
  circle_params.bg_color.blue = 0.0;
  circle_params.bg_color.alpha = 0.3;
  circle_params.has_bg_color = 1;

  display_meta->num_circles += 1;
  nvds_add_display_meta_to_frame(frame_meta, display_meta);
  RCLCPP_DEBUG(this->get_logger(), "Drew circle at (%d, %d)", circle_x_,
               circle_y_);
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

  GstElement *render = get_element("render");
  if (!render) {
    RCLCPP_ERROR(get_logger(), "Failed to get nvosd element.");
    return false;
  }

  GstPad *sink_pad = gst_element_get_static_pad(render, "sink");
  gst_object_unref(render);

  if (!sink_pad) {
    RCLCPP_ERROR(get_logger(), "Failed to get nvosd sink pad.");
    return false;
  }

  gst_pad_add_probe(
      sink_pad, GST_PAD_PROBE_TYPE_BUFFER,
      [](GstPad *, GstPadProbeInfo *info,
         gpointer user_data) -> GstPadProbeReturn {
        auto *self = static_cast<RtpClientNode *>(user_data);

        GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
        if (!buf) {
          RCLCPP_ERROR(self->get_logger(),
                       "Failed to get buffer from probe info.");
          return GST_PAD_PROBE_OK;
        }

        NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buf);
        if (!batch_meta) {
          RCLCPP_ERROR(self->get_logger(),
                       "Failed to get batch meta from buffer.");
          return GST_PAD_PROBE_OK;
        }

        for (NvDsMetaList *l_frame = batch_meta->frame_meta_list;
             l_frame != nullptr; l_frame = l_frame->next) {
          auto *frame_meta = static_cast<NvDsFrameMeta *>(l_frame->data);
          self->draw_circle(batch_meta, frame_meta);
        }

        return GST_PAD_PROBE_OK;
      },
      this, nullptr);

  gst_object_unref(sink_pad);

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