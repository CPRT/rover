#include "capture_node.hpp"
#include <fstream>
#include <gst/app/gstappsink.h>
#include <gst/gst.h>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sstream>
#include <vector>

VideoCaptureNode::VideoCaptureNode(const rclcpp::NodeOptions &options)
    : Node("video_capture_node", options) {
  this->declare_parameter<std::string>("listen_to", "detect");
  listen_to_ = this->get_parameter("listen_to").as_string();

  service_ = this->create_service<interfaces::srv::VideoCapture>(
      "/capture_frame",
      std::bind(&VideoCaptureNode::handle_capture, this, std::placeholders::_1,
                std::placeholders::_2));
}

void VideoCaptureNode::handle_capture(
    const std::shared_ptr<interfaces::srv::VideoCapture::Request> request,
    std::shared_ptr<interfaces::srv::VideoCapture::Response> response) {
  std::stringstream pipeline_desc;
  pipeline_desc << "interpipesrc listen-to=" << listen_to_
                << " is-live=true allow-renegotiation=false ! "
                   "queue ! nvvidconv ! videoconvert ! "
                   "jpegenc ! appsink name=sink emit-signals=true "
                   "max-buffers=1 drop=true sync=false";

  GError *err = nullptr;
  GstElement *pipeline = gst_parse_launch(pipeline_desc.str().c_str(), &err);

  if (!pipeline || err) {
    RCLCPP_ERROR(this->get_logger(), "Pipeline creation failed: %s",
                 err ? err->message : "unknown");
    response->success = false;
    return;
  }

  GstElement *appsink = gst_bin_get_by_name(GST_BIN(pipeline), "sink");

  gst_element_set_state(pipeline, GST_STATE_PLAYING);

  // Pull one sample (blocking)
  GstSample *sample = gst_app_sink_pull_sample(GST_APP_SINK(appsink));

  if (!sample) {
    RCLCPP_ERROR(this->get_logger(), "Failed to pull sample");
    response->success = false;
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsink);
    gst_object_unref(pipeline);
    return;
  }

  GstBuffer *buffer = gst_sample_get_buffer(sample);
  GstMapInfo map;

  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    RCLCPP_ERROR(this->get_logger(), "Buffer map failed");
    response->success = false;
    gst_sample_unref(sample);
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_object_unref(appsink);
    gst_object_unref(pipeline);
    return;
  }

  // Fill ROS message
  response->image.format = "jpeg";
  response->image.data.assign(map.data, map.data + map.size);

  gst_buffer_unmap(buffer, &map);
  gst_sample_unref(sample);

  response->success = true;

  // Optional file save
  if (!request->filename.empty()) {
    std::ofstream file(request->filename, std::ios::binary);
    file.write(reinterpret_cast<char *>(response->image.data.data()),
               response->image.data.size());
  }
  gst_element_set_state(pipeline, GST_STATE_NULL);
  gst_object_unref(appsink);
  gst_object_unref(pipeline);
}

RCLCPP_COMPONENTS_REGISTER_NODE(VideoCaptureNode)