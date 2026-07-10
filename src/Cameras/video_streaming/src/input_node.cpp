#include "input_node.hpp"
#include <cmath>
#include <filesystem>
#include <rclcpp_components/register_node_macro.hpp>

InputNode::InputNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("input_node", options) {
  RCLCPP_INFO(this->get_logger(), "InputNode constructed.");
  declare_parameters();
  video_service_ = this->create_service<interfaces::srv::VideoOut>(
      "start_video", std::bind(&InputNode::video_cb, this,
                               std::placeholders::_1, std::placeholders::_2));
  get_cam_service_ = create_service<interfaces::srv::GetCameras>(
      "~/get_cameras",
      [this](
          const std::shared_ptr<interfaces::srv::GetCameras::Request> request,
          std::shared_ptr<interfaces::srv::GetCameras::Response> response) {
        this->get_parameter("camera_name", response->sources);
      });

  start_pipeline();
}

void InputNode::declare_parameters() {
  this->declare_parameter("out_width", 1920);
  this->declare_parameter("out_height", 1080);
  this->declare_parameter("out_framerate", 30);
  this->declare_parameter("camera_name", std::vector<std::string>());
  std::vector<std::string> camera_name;
  this->get_parameter("camera_name", camera_name);
  for (const auto &name : camera_name) {
    this->declare_parameter(name + ".path", std::string());
    this->declare_parameter(name + ".type",
                            static_cast<int>(CameraType::V4l2Src));
    this->declare_parameter(name + ".encoded", false);
    this->declare_parameter(name + ".width", 0);
    this->declare_parameter(name + ".height", 0);
    this->declare_parameter(name + ".framerate", 0);
  }
}

bool InputNode::create_pipeline() {
  std::stringstream desc;
  const int W = this->get_parameter("out_width").as_int();
  const int H = this->get_parameter("out_height").as_int();
  const int FPS = this->get_parameter("out_framerate").as_int();

  // Dummy black source → tee → feeds both compositors (keeps them alive with
  // no cameras and provides a fixed-size reference pad at sink_0)
  desc << "videotestsrc is-live=true pattern=black "
       << "! video/x-raw,width=" << W << ",height=" << H << ",framerate=" << FPS
       << "/1 " << "! nvvidconv ! tee name=t0 "
       << "t0. ! queue leaky=downstream max-size-buffers=1 ! compositor.sink_0 "
       << "t0. ! queue leaky=downstream max-size-buffers=1 ! "
          "mosaic_compositor.sink_0 ";

  size_t index = 1;
  std::vector<std::string> camera_name;
  this->get_parameter("camera_name", camera_name);
  for (const auto &name : camera_name) {
    std::string path;
    this->get_parameter(name + ".path", path);
    int type_int;
    this->get_parameter(name + ".type", type_int);
    CameraType type = static_cast<CameraType>(type_int);

    if (type == CameraType::TestSrc) {
      desc << "videotestsrc is-live=true name=" << name << " ! ";
    } else if (type == CameraType::V4l2Src) {
      if (!std::filesystem::exists(path)) {
        RCLCPP_ERROR(this->get_logger(), "Camera path does not exist: %s",
                     path.c_str());
        continue;
      }
      desc << "v4l2src device=" << path << " name=" << name << " ! ";
      bool encoded;
      this->get_parameter(name + ".encoded", encoded);
      if (encoded) {
        desc << "jpegdec ! ";
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "Unimplemented Type for camera: %s",
                  name.c_str());
      continue;
    }

    int cam_width = this->get_parameter(name + ".width").as_int();
    int cam_height = this->get_parameter(name + ".height").as_int();
    int cam_framerate = this->get_parameter(name + ".framerate").as_int();
    if (cam_width > 0 || cam_height > 0 || cam_framerate > 0) {
      desc << "video/x-raw,";
      if (cam_width > 0)
        desc << "width=" << cam_width << ",";
      if (cam_height > 0)
        desc << "height=" << cam_height << ",";
      if (cam_framerate > 0)
        desc << "framerate=" << cam_framerate << "/1,";
      // Remove trailing comma
      std::string s = desc.str();
      if (s.back() == ',') {
        s.pop_back();
        desc.str("");
        desc << s << " ! ";
      }
    }

    // Split each camera via tee into both compositors
    desc << "nvvidconv ! tee name=t" << index << " " << "t" << index
         << ". ! queue leaky=downstream max-size-buffers=1 "
         << "! compositor.sink_" << index << " " << "t" << index
         << ". ! queue leaky=downstream max-size-buffers=1 "
         << "! mosaic_compositor.sink_" << index << " ";
    source_map_.emplace(name, index);
    ++index;
  }

  // Stream 1 – operator compositor: layout is set dynamically via video_cb
  desc << "nvcompositor name=compositor sink_0::alpha=0.0 ! "
       << "nvvidconv ! videorate ! " << "video/x-raw,width=" << W
       << ",height=" << H << ",framerate=" << FPS << "/1 "
       << "! interpipesink name=input ";

  // Stream 2 – mosaic compositor: fixed grid layout set by set_mosaic_layout()
  desc << "nvcompositor name=mosaic_compositor sink_0::alpha=0.0 ! "
       << "nvvidconv ! videorate ! " << "video/x-raw,width=" << W
       << ",height=" << H << ",framerate=" << FPS << "/1 "
       << "! interpipesink name=mosaic";

  RCLCPP_INFO(this->get_logger(), "Pipeline description: %s",
              desc.str().c_str());
  GError *err = nullptr;
  GstElement *p = gst_parse_launch(desc.str().c_str(), &err);
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
  set_mosaic_layout();

  if (current_video_request_) {
    video_cb(current_video_request_,
             std::make_shared<interfaces::srv::VideoOut::Response>());
  }
  return true;
}

bool InputNode::set_mosaic_layout() {
  const int n = static_cast<int>(source_map_.size());
  if (n == 0)
    return true;

  GstElement *mosaic = get_element("mosaic_compositor");
  if (!mosaic) {
    RCLCPP_ERROR(this->get_logger(),
                 "set_mosaic_layout: mosaic_compositor not found");
    return false;
  }

  const int W = this->get_parameter("out_width").as_int();
  const int H = this->get_parameter("out_height").as_int();
  const int cols =
      static_cast<int>(std::ceil(std::sqrt(static_cast<double>(n))));
  const int rows = (n + cols - 1) / cols;
  const int cell_w = W / cols;
  const int cell_h = H / rows;

  int grid_i = 0;
  for (const auto &[cam_name, sink_idx] : source_map_) {
    const int col = grid_i % cols;
    const int row = grid_i / cols;
    const std::string pad_name = "sink_" + std::to_string(sink_idx);
    GstPad *pad = gst_element_get_static_pad(mosaic, pad_name.c_str());
    if (!pad) {
      RCLCPP_WARN(this->get_logger(), "set_mosaic_layout: pad %s not found",
                  pad_name.c_str());
      ++grid_i;
      continue;
    }
    g_object_set(G_OBJECT(pad), "xpos", col * cell_w, "ypos", row * cell_h,
                 "width", cell_w, "height", cell_h, "alpha", 1.0, "zorder",
                 grid_i, NULL);
    gst_object_unref(pad);
    ++grid_i;
  }

  gst_object_unref(mosaic);
  RCLCPP_INFO(this->get_logger(),
              "Mosaic layout: %d cameras, %d cols x %d rows, cell %dx%d px", n,
              cols, rows, cell_w, cell_h);
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
    std::string pad_name = "sink_" + std::to_string(i + 1);
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
