#include "rtp_node.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/int32.hpp"
#include <glib.h>
#include <gst/gststructure.h>
#include <gst/video/video-event.h>
#include <rclcpp_components/register_node_macro.hpp>
namespace video_streaming {

RtpNode::RtpNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("rtp_node", options) {
  RCLCPP_INFO(this->get_logger(), "Initializing RtpNode...");

  this->declare_parameter<std::string>("dest_ip", "127.0.0.1");
  this->declare_parameter<int>("dest_port", 7001);
  this->declare_parameter<int>("target_framerate", 30);
  this->declare_parameter<int>("bitrate", 2000000);
  this->declare_parameter<int>("fec_percent", 20);
  this->declare_parameter<int>("default_width", 1280);
  this->declare_parameter<int>("default_height", 720);

  target_framerate_ = this->get_parameter("target_framerate").as_int();
  if (target_framerate_ <= 0) {
    RCLCPP_ERROR(this->get_logger(),
                 "Invalid initial target_framerate: %d. Must be > 0.",
                 target_framerate_);
    throw std::runtime_error("Invalid initial target_framerate");
  }
  bitrate_ = this->get_parameter("bitrate").as_int();
  fec_ = this->get_parameter("fec_percent").as_int();
  dest_ip_ = this->get_parameter("dest_ip").as_string();
  dest_port_ = this->get_parameter("dest_port").as_int();
  width_ = this->get_parameter("default_width").as_int();
  height_ = this->get_parameter("default_height").as_int();
  h265_encoder_ = nullptr;

  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&RtpNode::on_parameter_change, this, std::placeholders::_1));

  bitrate_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "~/set_bitrate", 10, [this](const std_msgs::msg::Int32::SharedPtr msg) {
        set_bitrate(msg->data);
      });

  iframe_sub_ = this->create_subscription<std_msgs::msg::Empty>(
      "~/trigger_iframe", 10,
      std::bind(&RtpNode::on_iframe_trigger, this, std::placeholders::_1));
  restart_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/all_video/restart_pipeline", 10);

  if (!start_pipeline()) {
    RCLCPP_FATAL(get_logger(), "RtpNode: failed to start pipeline.");
    throw std::runtime_error("RtpNode: pipeline start failed");
  }
  RCLCPP_INFO(get_logger(), "RtpNode constructed and pipeline started.");
}

RtpNode::~RtpNode() { BaseVideoNode::safe_gst_unref(h265_encoder_); }

std::string RtpNode::get_pipeline_description() {
  int video_bitrate = bitrate_ / (1 + fec_ / 100.0);
  std::stringstream desc;
  desc << "interpipesrc format=3 listen-to=detect is-live=true ! "
       << "nvvidconv ! videorate ! "
       << "capsfilter caps=\"video/x-raw,framerate=" << target_framerate_
       << "/1\" ! nvvidconv ! "
       << "capsfilter caps=\"video/x-raw(memory:NVMM),height=" << height_
       << ",width=" << width_ << "\" ! nvv4l2h265enc bitrate=" << video_bitrate
       << " name=h265_enc EnableTwopassCBR=true control-rate=1 "
          "maxperf-enable=true ! queue ! rtph265pay  config-interval=-1 ! "
          "rtpulpfecenc percentage="
       << fec_ << " ! udpsink host=" << dest_ip_ << " port=" << dest_port_;
  return desc.str();
}

bool RtpNode::create_pipeline() {
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
  // Unref any existing elements before getting new ones
  safe_gst_unref(h265_encoder_);
  h265_encoder_ = this->get_element("h265_enc");

  if (!h265_encoder_) {
    RCLCPP_ERROR(get_logger(), "Failed to get elements by name");
    return false;
  }

  RCLCPP_INFO(this->get_logger(),
              "Pipeline parsed and elements retrieved successfully.");
  return true;
}

rcl_interfaces::msg::SetParametersResult
RtpNode::on_parameter_change(const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  bool needs_restart = false;

  for (const auto &param : parameters) {
    const std::string &name = param.get_name();

    // Forward error correction
    if (name == "fec_percent") {
      fec_ = param.as_int();
      RCLCPP_INFO(this->get_logger(),
                  "Param Update: Forward Error Correction set to %d%%", fec_);
      needs_restart = true;
      continue;
    }
    // Framerate
    if (name == "target_framerate") {
      target_framerate_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Param Update: target_framerate = %d",
                  target_framerate_);
      if (target_framerate_ <= 0) {
        result.successful = false;
        result.reason = "target_framerate must be greater than 0";
        RCLCPP_ERROR(this->get_logger(),
                     "Invalid target_framerate: %d. Must be > 0.",
                     target_framerate_);
        return result;
      }
      needs_restart = true;
      continue;
    }
    // Destination IP
    if (name == "dest_ip") {
      dest_ip_ = param.as_string();
      RCLCPP_INFO(this->get_logger(), "Param Update: dest_ip = %s",
                  dest_ip_.c_str());
      needs_restart = true;
      continue;
    }
    // Destination Port
    if (name == "dest_port") {
      dest_port_ = param.as_int();
      RCLCPP_INFO(this->get_logger(), "Param Update: dest_port = %d",
                  dest_port_);
      needs_restart = true;
      continue;
    }
    // Bitrate
    if (name == "bitrate") {
      int bitrate = param.as_int();
      set_bitrate(bitrate);
      RCLCPP_INFO(this->get_logger(), "Param Update: bitrate = %d", bitrate);
      RCLCPP_WARN_ONCE(this->get_logger(),
                       "Bitrate changes should be made via the "
                       "~/set_bitrate topic for smoother transitions.");
      continue;
    }
  }
  if (needs_restart && result.successful) {
    restart_pub_->publish(std_msgs::msg::Empty());
  }
  return result;
}

void RtpNode::set_bitrate(const int32_t bitrate) {
  bitrate_ = bitrate;
  int video_bitrate = bitrate / (1 + fec_ / 100.0);
  if (h265_encoder_) {
    pause_pipeline();
    g_object_set(h265_encoder_, "bitrate", video_bitrate, NULL);
    resume_pipeline();
    RCLCPP_DEBUG(this->get_logger(), "Bitrate set to %d", video_bitrate);
  }
  // Adjust resolution based on bitrate thresholds
  // TODO: These thresholds are arbitrary and may need tuning based on actual
  // performance and quality requirements
  int bits_per_frame = video_bitrate / target_framerate_;
  if (bits_per_frame < 50000) {
    set_resolution(640, 360);
  } else if (bits_per_frame < 100000) {
    set_resolution(854, 480);
  } else if (bits_per_frame < 200000) {
    set_resolution(1280, 720);
  } else {
    set_resolution(1920, 1080);
  }
}

void RtpNode::set_resolution(const int width, const int height) {
  if (width == width_ && height == height_) {
    return;
  }
  width_ = width;
  height_ = height;
  restart_pub_->publish(std_msgs::msg::Empty());
}

void RtpNode::on_iframe_trigger(const std_msgs::msg::Empty::SharedPtr msg) {
  if (h265_encoder_) {
    g_signal_emit_by_name(h265_encoder_, "force-IDR");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(video_streaming::RtpNode);
} // namespace video_streaming