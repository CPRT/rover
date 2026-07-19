#include "zoom_node.hpp"
#include <cstdint>
#include <rclcpp_components/register_node_macro.hpp>

ZoomNode::ZoomNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("zoom_node", options) {
  this->declare_parameter<std::string>("listen_to", "input");
  this->declare_parameter<std::string>("output_to", "zoom");
  this->declare_parameter<int>("height", 1080);
  this->declare_parameter<int>("width", 1920);
  zoom_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "zoom", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      std::bind(&ZoomNode::zoom_cb, this, std::placeholders::_1));
  pan_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "pan", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&ZoomNode::pan_cb, this, std::placeholders::_1));
  tilt_sub_ = this->create_subscription<std_msgs::msg::Int8>(
      "tilt", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
      std::bind(&ZoomNode::tilt_cb, this, std::placeholders::_1));
  start_pipeline();
}

bool ZoomNode::create_pipeline() {
  std::stringstream desc_ss;
  desc_ss << "interpipesrc format=3 listen-to="
          << this->get_parameter("listen_to").as_string()
          << " is-live=true "
             "allow-renegotiation=true name=src ! ";
  desc_ss << "nvvideoconvert ! capsfilter caps=\""
          << "video/x-raw(memory:NVMM),format=NV12,width="
          << this->get_parameter("width").as_int()
          << ",height=" << this->get_parameter("height").as_int() << "\" ! ";
  desc_ss << "nvvideoconvert name=zoomer src-crop=\"" << crop_.to_string()
          << "\" ! " << "interpipesink name=";
  desc_ss << this->get_parameter("output_to").as_string();

  RCLCPP_INFO(this->get_logger(), "Creating pipeline: %s",
              desc_ss.str().c_str());

  GError *err = nullptr;
  GstElement *p = gst_parse_launch(desc_ss.str().c_str(), &err);
  if (err || !p) {
    RCLCPP_ERROR(this->get_logger(), "gst_parse_launch failed: %s",
                 err ? err->message : "unknown");
    if (err)
      g_error_free(err);
    return false;
  }
  if (!GST_IS_PIPELINE(p)) {
    RCLCPP_ERROR(this->get_logger(), "Parsed element is not a pipeline.");
    gst_object_unref(p);
    return false;
  }
  if (pipeline_) {
    RCLCPP_ERROR(this->get_logger(),
                 "Pipeline already exists in create_pipeline.");
    gst_object_unref(p);
    return false;
  }

  pipeline_ = p;
  return true;
}

void ZoomNode::update() {
  uint32_t total_height =
      static_cast<uint32_t>(this->get_parameter("height").as_int());
  uint32_t total_width =
      static_cast<uint32_t>(this->get_parameter("width").as_int());
  uint32_t crop_from_height =
      static_cast<uint32_t>(total_height * (zoom_ - 1.0f) / (2.0f * zoom_));
  uint32_t crop_from_width =
      static_cast<uint32_t>(total_width * (zoom_ - 1.0f) / (2.0f * zoom_));
  crop_.top = static_cast<uint16_t>(crop_from_height * (tilt_ / 100.0f));
  crop_.bottom =
      static_cast<uint16_t>(crop_from_height * (1.0f - (tilt_ / 100.0f)));
  crop_.left = static_cast<uint16_t>(crop_from_width * (pan_ / 100.0f));
  crop_.right =
      static_cast<uint16_t>(crop_from_width * (1.0f - (pan_ / 100.0f)));
  GstElement *zoomer = get_element("zoomer");
  if (zoomer) {
    g_object_set(zoomer, "src-crop", crop_.to_string().c_str(), nullptr);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Could not find zoomer element.");
  }
}

void ZoomNode::zoom_cb(const std_msgs::msg::Float32::SharedPtr msg) {
  float zoom = msg->data;
  if (zoom < 1.0f) {
    zoom = 1.0f;
  }
  zoom_ = zoom;
  update();
}

void ZoomNode::pan_cb(const std_msgs::msg::Int8::SharedPtr msg) {
  int8_t pan = msg->data;
  if (pan < 0) {
    pan = 0;
  } else if (pan > 100) {
    pan = 100;
  }
  pan_ = pan;
  update();
}

void ZoomNode::tilt_cb(const std_msgs::msg::Int8::SharedPtr msg) {
  int8_t tilt = msg->data;
  if (tilt < 0) {
    tilt = 0;
  } else if (tilt > 100) {
    tilt = 100;
  }
  tilt_ = tilt;
  update();
}

RCLCPP_COMPONENTS_REGISTER_NODE(ZoomNode)
