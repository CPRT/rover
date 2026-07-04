#include "mosaic_webrtc_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>

namespace video_streaming {

MosaicWebRtcNode::MosaicWebRtcNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("mosaic_webrtc_node", options) {
  // Allow the signalling port to be overridden at launch time.
  this->declare_parameter<int>("signalling_port", 8444);
  this->declare_parameter<int>("bitrate", 1000000);
  this->declare_parameter<int>("framerate", 5);

  if (!start_pipeline()) {
    RCLCPP_FATAL(get_logger(), "MosaicWebRtcNode: failed to start pipeline.");
    throw std::runtime_error("MosaicWebRtcNode: pipeline start failed");
  }
  RCLCPP_INFO(get_logger(), "MosaicWebRtcNode: pipeline started (signalling port %ld).",
              this->get_parameter("signalling_port").as_int());
}

bool MosaicWebRtcNode::create_pipeline() {
  const int port = this->get_parameter("signalling_port").as_int();
  const int bitrate = this->get_parameter("bitrate").as_int();
  const int framerate = this->get_parameter("framerate").as_int();
  std::ostringstream desc;
  // All-intra H.264 (iframeinterval=1): every frame is a keyframe, so the
  // stream behaves like MJPEG (frame drops never corrupt the picture) while
  // compressing better than JPEG. Encoding explicitly here instead of letting
  // webrtcsink pick an encoder keeps the encoder settings under our control.
  desc << "interpipesrc listen-to=mosaic is-live=true ! "
       << "videorate drop-only=true ! video/x-raw,framerate=" << framerate
       << "/1 ! nvvidconv ! "
       << "nvv4l2h264enc iframeinterval=1 insert-sps-pps=true control-rate=1 "
       << "bitrate=" << bitrate << " ! "
       << "h264parse config-interval=-1 ! "
       << "webrtcsink run-signalling-server=true "
       << "signalling-server-port=" << port << " "
       << "video-caps=\"video/x-h264\"";

  RCLCPP_INFO(get_logger(), "MosaicWebRtcNode pipeline: %s", desc.str().c_str());
  GError *err = nullptr;
  GstElement *p = gst_parse_launch(desc.str().c_str(), &err);
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

RCLCPP_COMPONENTS_REGISTER_NODE(video_streaming::MosaicWebRtcNode)
} // namespace video_streaming
