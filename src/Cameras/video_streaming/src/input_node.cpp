#include "input_node.hpp"
#include <filesystem>
#include <rclcpp_components/register_node_macro.hpp>

InputNode::InputNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("input_node", options) {
  RCLCPP_INFO(this->get_logger(), "InputNode constructed.");
  declare_parameters();
  video_service_ = this->create_service<interfaces::srv::VideoOut>(
      "start_video", std::bind(&InputNode::video_cb, this,
                               std::placeholders::_1, std::placeholders::_2));
  start_pipeline();
}

void InputNode::declare_parameters() {
  this->declare_parameter("out_width", 1280);
  this->declare_parameter("out_height", 720);
  this->declare_parameter("out_framerate", 30);
  this->declare_parameter("camera_name", std::vector<std::string>());
  std::vector<std::string> camera_name;
  this->get_parameter("camera_name", camera_name);
  for (const auto &name : camera_name) {
    this->declare_parameter(name + ".path", std::string());
    this->declare_parameter(name + ".type",
                            static_cast<int>(CameraType::V4l2Src));
    this->declare_parameter(name + ".encoded", false);
  }
}

bool InputNode::create_pipeline() {
  std::stringstream desc_stream;
  size_t index = 0;
  std::vector<std::string> camera_name;
  this->get_parameter("camera_name", camera_name);
  for (const auto &name : camera_name) {
    std::string path;
    this->get_parameter(name + ".path", path);
    int type_int;
    this->get_parameter(name + ".type", type_int);
    CameraType type = static_cast<CameraType>(type_int);

    if (type == CameraType::TestSrc) {
      desc_stream << "videotestsrc is-live=true name=" << name << " ! ";
    } else if (type == CameraType::V4l2Src) {
      if (!std::filesystem::exists(path)) {
        RCLCPP_ERROR(this->get_logger(), "Camera path does not exist: %s",
                     path.c_str());
        continue;
      }
      desc_stream << "v4l2src device=" << path << " name=" << name << " ! ";
      bool encoded;
      this->get_parameter(name + ".encoded", encoded);
      if (encoded) {
        desc_stream << "jpegdec ! ";
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Unimplemented Type for camera: %s",
                  name.c_str());
      continue;
    }
    desc_stream << "nvvidconv ! compositor.sink_" << index << " ";
    source_map_.emplace(name, index);
    ++index;
  }
  desc_stream << "nvcompositor name=compositor sink_0::width="
              << this->get_parameter("out_width").as_int() << " sink_0::height="
              << this->get_parameter("out_height").as_int();
  desc_stream << " ! nvvidconv ! videorate ! video/x-raw,width="
              << this->get_parameter("out_width").as_int()
              << ",height=" << this->get_parameter("out_height").as_int()
              << ",framerate=" << this->get_parameter("out_framerate").as_int()
              << "/1 ! interpipesink name=input";

  RCLCPP_INFO(this->get_logger(), "Pipeline description: %s",
              desc_stream.str().c_str());
  GError *err = nullptr;
  GstElement *p = gst_parse_launch(desc_stream.str().c_str(), &err);
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

  pipeline_ = p;
  if (current_video_request_) {
    video_cb(current_video_request_,
             std::make_shared<interfaces::srv::VideoOut::Response>());
  }
  return true;
}

void InputNode::video_cb(
    const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
    std::shared_ptr<interfaces::srv::VideoOut::Response> response) {
  if (!pipeline_) {
    RCLCPP_ERROR(this->get_logger(), "Pipeline not initialized");
    response->success = false;
    return;
  }
  RCLCPP_INFO(this->get_logger(), "Received request");
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    gst_element_set_state(pipeline_, GST_STATE_PAUSED);
  }

  const auto total_height = this->get_parameter("out_height").as_int();
  const auto total_width = this->get_parameter("out_width").as_int();

  GstElement *compositor = get_element("compositor");

  if (!compositor) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get compositor element");
    response->success = false;
    return;
  }
  for (size_t i = 0; i < source_map_.size(); ++i) {
    std::string pad_name = "sink_" + std::to_string(i);
    GstPad *pad = gst_element_get_static_pad(compositor, pad_name.c_str());
    if (!pad) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get compositor pad: %s",
                   pad_name.c_str());
      response->success = false;
      return;
    }
    g_object_set(G_OBJECT(pad), "alpha", 0, NULL);
    gst_object_unref(pad);
  }
  size_t order = 0;
  for (const auto &source : request->sources) {
    const std::string &name = source.name;
    const int height = source.height * total_height / 100;
    const int width = source.width * total_width / 100;
    const int origin_x = source.origin_x * total_width / 100;
    const int origin_y = source.origin_y * total_height / 100;

    auto iter = source_map_.find(name);
    if (iter == source_map_.end()) {
      RCLCPP_WARN(this->get_logger(), "%s: Could not find camera %s",
                  __FUNCTION__, name.c_str());
      continue;
    }
    const std::string pad_name = "sink_" + std::to_string(iter->second);
    GstPad *pad = gst_element_get_static_pad(compositor, pad_name.c_str());
    if (!pad) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get compositor pad: %s",
                   pad_name.c_str());
      response->success = false;
      return;
    }
    g_object_set(G_OBJECT(pad), "xpos", origin_x, "ypos", origin_y, "height",
                 height, "width", width, "zorder", order++, "alpha", 1.0, NULL);
    gst_object_unref(pad);
  }
  gst_object_unref(compositor);
  GstStateChangeReturn ret;
  {
    std::lock_guard<std::mutex> lock(pipeline_mutex_);
    ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
  }
  if (ret != GST_STATE_CHANGE_FAILURE) {
    RCLCPP_INFO(this->get_logger(), "Pipeline started successfully");
    response->success = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
    response->success = false;
  }
  if (response->success) {
    current_video_request_ = request;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(InputNode)
