#include "webrtc_node.h"

#include <gst/app/gstappsink.h>

#include <filesystem>
#include <fstream>

#include "rclcpp/executors.hpp"

WebRTCStreamer::WebRTCStreamer()
    : Node("webrtc_node"), pipeline_(nullptr), compositor_(nullptr) {
  gst_init(nullptr, nullptr);

  // Declare parameters
  declare_parameters();

  this->get_parameter("web_server", web_server_);
  this->get_parameter("web_server_path", web_server_path_);
  this->get_parameter("max_width", width_);
  this->get_parameter("max_height", height_);
  this->get_parameter("max_framerate", framerate_);

  // Set up the service for starting video
  start_video_service_ = this->create_service<interfaces::srv::VideoOut>(
      "start_video", std::bind(&WebRTCStreamer::start_video_cb, this,
                               std::placeholders::_1, std::placeholders::_2));
  capture_service_ = this->create_service<interfaces::srv::VideoCapture>(
      "capture_frame", std::bind(&WebRTCStreamer::capture_frame, this,
                                 std::placeholders::_1, std::placeholders::_2));
  get_cams_service_ = this->create_service<interfaces::srv::GetCameras>(
      "get_cameras", std::bind(&WebRTCStreamer::get_cameras, this,
                               std::placeholders::_1, std::placeholders::_2));
  restart_service_ = this->create_service<std_srvs::srv::Trigger>(
      "reset_video", std::bind(&WebRTCStreamer::restart_pipeline, this,
                               std::placeholders::_1, std::placeholders::_2));
  marker_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "marker_detected", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());

  start();
  GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_.get()), GST_DEBUG_GRAPH_SHOW_ALL,
                            "start_pipeline");
}

WebRTCStreamer::~WebRTCStreamer() { stop(); }

bool WebRTCStreamer::start() {
  // Fetch camera parameters
  std::vector<std::string> camera_names;
  this->get_parameter("camera_name", camera_names);
  initialize_pipeline();

  for (const auto &name : camera_names) {
    std::string camera_path;
    this->get_parameter(name + ".path", camera_path);
    int camera_type;
    this->get_parameter(name + ".type", camera_type);
    bool encoded;
    this->get_parameter(name + ".encoded", encoded);
    bool aruco;
    this->get_parameter(name + ".aruco", aruco);
    CameraSource source;
    source.name = name;
    source.path = camera_path;
    source.type = static_cast<CameraType>(camera_type);
    source.encoded = encoded;
    source.aruco = aruco;

    if (source.type != CameraType::TestSrc && camera_path.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Camera path not set for %s",
                   name.c_str());
      continue;
    }
    if (source.type == CameraType::V4l2Src &&
        !std::filesystem::exists(source.path)) {
      RCLCPP_ERROR(this->get_logger(), "Camera path does not exist: %s",
                   camera_path.c_str());
      continue;
    }
    GstElement *src = create_source(source);
    if (src) {
      auto src_pad =
          GstUniquePtr<GstPad>(gst_element_get_static_pad(src, "src"));
      auto sink_pad = GstUniquePtr<GstPad>(
          gst_element_request_pad_simple(compositor_, "sink_%u"));
      g_object_set(G_OBJECT(sink_pad.get()), "height", height_, "width", width_,
                   "alpha", 0.0, NULL);
      auto rc = gst_pad_link(src_pad.get(), sink_pad.get());
      if (rc != GST_PAD_LINK_OK) {
        RCLCPP_ERROR(this->get_logger(), "Could not link camera %s",
                     name.c_str());
        gst_element_release_request_pad(compositor_, sink_pad.get());
        continue;
      }
      source_pads_.emplace(name, std::move(sink_pad));
      RCLCPP_INFO(this->get_logger(), "Camera source %s created", name.c_str());
    }
  }
  const auto ret = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
  if (ret == GST_STATE_CHANGE_FAILURE) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "Pipeline started successfully");
  return true;
}

void WebRTCStreamer::stop() {
  for (auto &pair : source_pads_) {
    gst_element_release_request_pad(compositor_, pair.second.get());
  }
  source_pads_.clear();

  for (const auto &element : elements_) {
    if (element) {
      gst_element_set_state(element, GST_STATE_NULL);
      gst_bin_remove(GST_BIN(pipeline_.get()), element);
      gst_object_unref(element);
    }
  }
  elements_.clear();

  if (pipeline_) {
    gst_element_set_state(pipeline_.get(), GST_STATE_NULL);
  }
}

void WebRTCStreamer::declare_parameters() {
  this->declare_parameter("web_server", true);
  this->declare_parameter("web_server_path", ".");
  this->declare_parameter("max_width", 1280);
  this->declare_parameter("max_height", 720);
  this->declare_parameter("max_framerate", 30);
  this->declare_parameter("camera_name", std::vector<std::string>());
  std::vector<std::string> camera_name;
  this->get_parameter("camera_name", camera_name);
  for (const auto &name : camera_name) {
    this->declare_parameter(name + ".path", std::string());
    this->declare_parameter(name + ".type",
                            static_cast<int>(CameraType::V4l2Src));
    this->declare_parameter(name + ".encoded", false);
    this->declare_parameter(name + ".aruco", false);
    this->declare_parameter(name + ".aruco_detect_interval", 1);
  }
}

void WebRTCStreamer::start_video_cb(
    const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
    std::shared_ptr<interfaces::srv::VideoOut::Response> response) {
  if (!pipeline_) {
    RCLCPP_ERROR(this->get_logger(), "Pipeline not initialized");
    response->success = false;
    return;
  }
  gst_element_set_state(pipeline_.get(), GST_STATE_PAUSED);
  if (update_pipeline(request) != true) {
    RCLCPP_ERROR(this->get_logger(), "Failed to update pipeline");
    response->success = false;
    GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_.get()),
                              GST_DEBUG_GRAPH_SHOW_ALL, "error_pipeline");
    return;
  }
  const auto ret = gst_element_set_state(pipeline_.get(), GST_STATE_PLAYING);
  if (ret != GST_STATE_CHANGE_FAILURE) {
    RCLCPP_INFO(this->get_logger(), "Pipeline started successfully");
    response->success = true;
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to start pipeline");
    response->success = false;
  }
  GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_.get()), GST_DEBUG_GRAPH_SHOW_ALL,
                            "pipeline_update");
}

void WebRTCStreamer::capture_frame(
    const std::shared_ptr<interfaces::srv::VideoCapture::Request> request,
    std::shared_ptr<interfaces::srv::VideoCapture::Response> response) {
  response->success = true;
  const std::string &name = request->source;
  auto source_tee = GstUniquePtr<GstElement>(
      gst_bin_get_by_name(GST_BIN(pipeline_.get()), name.c_str()));
  if (!source_tee) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get source: %s", name.c_str());
    return;
  }
  std::vector<GstElement *> elements;
  elements.emplace_back(create_element("queue"));
  if (!elements.back()) {
    return;
  }
  g_object_set(G_OBJECT(elements.back()), "max-size-buffers", 1, nullptr);
  g_object_set(G_OBJECT(elements.back()), "leaky", 2, nullptr);

  elements.emplace_back(create_vid_conv());
  elements.emplace_back(create_element("jpegenc"));

  GstElement *sink = create_element("appsink");
  elements.push_back(sink);
  if (!add_element_chain(elements)) {
    response->success = false;
    return;
  }

  if (!gst_element_link(source_tee.get(), elements.front())) {
    RCLCPP_ERROR(this->get_logger(),
                 "%s: Failed to link capture pipe section to %s", __FUNCTION__,
                 name.c_str());
    response->success = false;
    for (auto element : elements) {
      gst_bin_remove(GST_BIN(pipeline_.get()), element);
      gst_object_unref(element);
    }
    return;
  }
  constexpr auto timeout = 1000000000;  // 1 second
  GstSample *sample = gst_app_sink_try_pull_sample(GST_APP_SINK(sink), timeout);

  auto pad = GstUniquePtr<GstPad>(
      gst_element_get_static_pad(elements.front(), "sink"));
  unlink_pad(pad.get());
  for (auto element : elements) {
    gst_bin_remove(GST_BIN(pipeline_.get()), element);
  }

  if (!sample) {
    RCLCPP_ERROR(this->get_logger(), "Failed to pull sample from appsink");
    response->success = false;
    return;
  }
  GstBuffer *buffer = gst_sample_get_buffer(sample);
  if (!buffer) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get buffer from sample");
    gst_sample_unref(sample);
    response->success = false;
    return;
  }
  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_READ)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to map buffer");
    gst_sample_unref(sample);
    response->success = false;
    return;
  }
  std::vector<uint8_t> &img_vec = response->image.data;
  std::string &filename = request->filename;
  img_vec.resize(map.size);
  std::memcpy(img_vec.data(), map.data, map.size);
  response->image.format = "jpeg";
  if (!filename.empty()) {
    std::ofstream file(filename, std::ios::binary);
    if (!file) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open file %s for writing",
                   filename.c_str());
      response->success = false;
    } else {
      file.write(reinterpret_cast<const char *>(map.data), map.size);
      if (!file) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write to file %s",
                     filename.c_str());
        response->success = false;
      }
      file.close();
    }
  }
  gst_buffer_unmap(buffer, &map);
  gst_sample_unref(sample);
}

void WebRTCStreamer::get_cameras(
    const std::shared_ptr<interfaces::srv::GetCameras::Request> request,
    std::shared_ptr<interfaces::srv::GetCameras::Response> response) {
  response->sources = {};
  for (auto it = source_pads_.cbegin(); it != source_pads_.end(); ++it) {
    response->sources.push_back(it->first);
  }
}

void WebRTCStreamer::restart_pipeline(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  stop();
  response->success = start();
  if (response->success) {
    response->message = "Pipeline restarted successfully";
  } else {
    response->message = "Failed to restart pipeline";
  }
  GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(pipeline_.get()), GST_DEBUG_GRAPH_SHOW_ALL,
                            "restart_pipeline");
  RCLCPP_INFO(this->get_logger(), "%s", response->message.c_str());
}

GstElement *WebRTCStreamer::create_vid_conv() {
  GstElement *videoconvert = create_element("nvvidconv");
  if (videoconvert) {
    return videoconvert;
  }
  RCLCPP_INFO(this->get_logger(),
              "Failed to create nvvidconv, using videoconvert instead");
  return create_element("videoconvert");
}
GstElement *WebRTCStreamer::create_element(std::string element_type,
                                           std::string element_name) {
  GstElement *element = nullptr;
  if (element_name.empty()) {
    element = gst_element_factory_make(element_type.c_str(), nullptr);
  } else {
    element =
        gst_element_factory_make(element_type.c_str(), element_name.c_str());
  }
  if (!element) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create %s",
                 element_type.c_str());
  }
  return element;
}

GstElement *WebRTCStreamer::add_element_chain(
    const std::vector<GstElement *> &chain) {
  GstElement *lastElement = nullptr;
  bool success = true;
  for (auto element : chain) {
    if (!element) {
      RCLCPP_ERROR(this->get_logger(), "%s: Bad chain", __FUNCTION__);
      success = false;
      break;
    }
    gst_bin_add(GST_BIN(pipeline_.get()), element);
    gst_element_sync_state_with_parent(element);
    if (!lastElement) {
      lastElement = element;
      continue;
    }
    if (!gst_element_link(lastElement, element)) {
      char *name1 = gst_element_get_name(lastElement);
      char *name2 = gst_element_get_name(element);
      RCLCPP_ERROR(this->get_logger(), "Could not link %s with %s", name1,
                   name2);
      g_free(name1);
      g_free(name2);
      success = false;
      break;
    }
    lastElement = element;
  }
  if (success) {
    elements_.insert(elements_.end(), chain.begin(), chain.end());
    return lastElement;
  }
  for (auto element : chain) {
    if (element) {
      gst_bin_remove(GST_BIN(pipeline_.get()), element);
      gst_object_unref(element);
    }
    if (element == lastElement) {
      break;
    }
  }
  return nullptr;
}

static void on_marker_detected(GstElement *element, gint marker_id,
                               gpointer user_data) {
  auto *marker_pub =
      static_cast<rclcpp::Publisher<std_msgs::msg::Int32> *>(user_data);
  std_msgs::msg::Int32 msg;
  msg.data = marker_id;
  RCLCPP_INFO(rclcpp::get_logger("webrtc_node"), "Marker detected with ID: %d",
              marker_id);
  if (!marker_pub) {
    RCLCPP_ERROR(
        rclcpp::get_logger("webrtc_node"),
        "Marker publisher is not initialized, cannot publish marker ID");
    return;
  }
  marker_pub->publish(msg);
}

GstElement *WebRTCStreamer::create_source(const CameraSource &src) {
  std::vector<GstElement *> elements;

  const std::string &name = src.name;
  const CameraType &type = src.type;
  const std::map<CameraType, std::string> source_map = {
      {CameraType::TestSrc, "videotestsrc"},
      {CameraType::V4l2Src, "v4l2src"},
      {CameraType::NetworkSrc, "udpsrc"},
  };

  // Add src element(s)
  const auto iter = source_map.find(type);
  if (iter == source_map.end()) {
    RCLCPP_WARN(this->get_logger(), "Unimplemented Type for camera: %s",
                name.c_str());
    return nullptr;
  }
  GstElement *source_element = create_element(iter->second);
  if (!source_element) {
    return nullptr;
  }
  elements.push_back(source_element);
  if (src.type == CameraType::TestSrc) {
    g_object_set(G_OBJECT(source_element), "pattern", 0, nullptr);
    g_object_set(G_OBJECT(source_element), "is-live", TRUE, nullptr);
  } else if (src.type == CameraType::V4l2Src) {
    g_object_set(G_OBJECT(source_element), "device", src.path.c_str(), nullptr);
  } else if (src.type == CameraType::NetworkSrc) {
    g_object_set(G_OBJECT(source_element), "uri", src.path.c_str(), nullptr);
    auto caps = GstUniquePtr<GstCaps>(gst_caps_from_string(
        "application/x-rtp, media=video, encoding-name=JPEG, payload=26, "
        "clock-rate=90000"));
    g_object_set(G_OBJECT(source_element), "caps", caps.get(), nullptr);
    elements.push_back(create_element("rtpjpegdepay"));
  }
  // Add jpeg decoders (if necessary)
  if (src.encoded) {
    elements.emplace_back(create_element("jpegparse"));
    elements.emplace_back(create_element("jpegdec"));
  }

  // Add small queue that drops oldest buffers
  elements.emplace_back(create_element("queue"));
  if (!elements.back()) {
    for (auto element : elements) {
      if (element) gst_object_unref(element);
    }
    return nullptr;
  }
  g_object_set(G_OBJECT(elements.back()), "max-size-buffers", 1, nullptr);
  g_object_set(G_OBJECT(elements.back()), "leaky", 2, nullptr);

  if (src.aruco) {
    elements.emplace_back(create_element("videoconvert"));
    elements.emplace_back(create_element("arucomarker"));
    if (!elements.back()) {
      elements.pop_back();
      RCLCPP_WARN(this->get_logger(),
                  "Failed to create aruco marker element, skipping aruco "
                  "processing");
    } else {
      int aruco_detect_interval;
      this->get_parameter(name + ".aruco_detect_interval",
                          aruco_detect_interval);
      g_object_set(G_OBJECT(elements.back()), "detect-every",
                   aruco_detect_interval, nullptr);
      g_signal_connect(elements.back(), "marker-detected",
                       G_CALLBACK(on_marker_detected), marker_pub_.get());
    }
    elements.emplace_back(create_element("videoconvert"));
  }

  // Add video converter
  elements.emplace_back(create_vid_conv());
  // Add tee to connect screen capture to
  elements.emplace_back(create_element("tee", name));

  // Add small queue for dataflow seperations
  elements.emplace_back(create_element("queue"));
  g_object_set(G_OBJECT(elements.back()), "max-size-buffers", 1, nullptr);
  g_object_set(G_OBJECT(elements.back()), "leaky", 2, nullptr);

  elements.emplace_back(create_vid_conv());

  return add_element_chain(elements);
}

GstElement *WebRTCStreamer::initialize_pipeline() {
  pipeline_ = GstUniquePtr<GstElement>(gst_pipeline_new("webrtc-pipeline"));
  std::vector<GstElement *> elements;
  compositor_ = create_element("nvcompositor");
  if (!compositor_) {
    RCLCPP_INFO(this->get_logger(),
                "Could not create nvcompositor, using compositor instead");
    compositor_ = create_element("compositor");
  }
  elements.push_back(compositor_);

  elements.push_back(create_element("queue"));
  g_object_set(G_OBJECT(elements.back()), "max-size-buffers", 1, nullptr);
  g_object_set(G_OBJECT(elements.back()), "leaky", 2, nullptr);

  elements.emplace_back(create_vid_conv());

  elements.emplace_back(create_element("capsfilter"));
  GstElement *capsfilter = elements.back();
  assert(capsfilter);
  std::stringstream ss;
  ss << "video/x-raw(memory:NVMM), height=" << height_ << ", width=" << width_
     << ", framerate=" << framerate_ << "/1";
  auto caps = GstUniquePtr<GstCaps>(gst_caps_from_string(ss.str().c_str()));
  g_object_set(G_OBJECT(capsfilter), "caps", caps.get(), nullptr);

  elements.emplace_back(create_element("webrtcsink", "webrtcsink"));
  GstElement *webrtcsink = elements.back();
  assert(webrtcsink != nullptr);
  g_object_set(G_OBJECT(webrtcsink), "run-signalling-server", TRUE, nullptr);
  auto video_caps =
      GstUniquePtr<GstCaps>(gst_caps_from_string("video/x-h265; video/x-h264"));
  g_object_set(G_OBJECT(webrtcsink), "video-caps", video_caps.get(), nullptr);
  if (web_server_) {
    g_object_set(G_OBJECT(webrtcsink), "run-web-server", TRUE, nullptr);
    g_object_set(G_OBJECT(webrtcsink), "web-server-host-addr",
                 "http://0.0.0.0:8080/", nullptr);
    g_object_set(G_OBJECT(webrtcsink), "web-server-directory",
                 web_server_path_.c_str(), nullptr);
  }

  if (!add_element_chain(elements)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add elements to pipeline");
    return nullptr;
  }

  bus_ =
      GstUniquePtr<GstBus>(gst_pipeline_get_bus(GST_PIPELINE(pipeline_.get())));
  if (gst_bus_add_watch(bus_.get(), &WebRTCStreamer::on_bus_message, this)) {
    RCLCPP_INFO(this->get_logger(), "Bus watch added successfully.");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to add bus watch.");
  }
  return pipeline_.get();
}
bool WebRTCStreamer::unlink_pad(GstPad *pad) {
  if (!pad) {
    return false;
  }
  auto peer = GstUniquePtr<GstPad>(gst_pad_get_peer(pad));
  if (peer) {
    if (GST_PAD_IS_SRC(peer.get())) {
      gst_pad_unlink(peer.get(), pad);
    } else {
      gst_pad_unlink(pad, peer.get());
    }
    auto parent = gst_pad_get_parent_element(peer.get());
    gst_element_release_request_pad(parent, peer.get());
    gst_object_unref(parent);
  }
  return true;
}

bool WebRTCStreamer::update_pipeline(
    const std::shared_ptr<interfaces::srv::VideoOut::Request> request) {
  if (!pipeline_) {
    RCLCPP_ERROR(this->get_logger(), "Pipeline not initialized");
    return false;
  }
  const auto &total_height = height_;
  const auto &total_width = width_;

  int i = 1;
  for (auto it = source_pads_.cbegin(); it != source_pads_.end(); ++it) {
    auto &pad = it->second;
    g_object_set(G_OBJECT(pad.get()), "alpha", 0, NULL);
  }
  for (const auto &source : request->sources) {
    const std::string &name = source.name;
    const int height = source.height * total_height / 100;
    const int width = source.width * total_width / 100;
    const int origin_x = source.origin_x * total_width / 100;
    const int origin_y = source.origin_y * total_height / 100;

    auto iter = source_pads_.find(name);
    if (iter == source_pads_.end()) {
      RCLCPP_WARN(this->get_logger(), "%s: Could not find camera %s",
                  __FUNCTION__, name.c_str());
      continue;
    }
    auto &pad = iter->second;
    g_object_set(G_OBJECT(pad.get()), "xpos", origin_x, "ypos", origin_y,
                 "height", height, "width", width, "zorder", i, "alpha", 1.0,
                 NULL);
    ++i;
  }
  return true;
}

gboolean WebRTCStreamer::on_bus_message(GstBus *bus, GstMessage *message,
                                        gpointer user_data) {
  WebRTCStreamer *streamer = static_cast<WebRTCStreamer *>(user_data);
  RCLCPP_INFO(streamer->get_logger(), "Received bus message: %s",
              GST_MESSAGE_TYPE_NAME(message));

  switch (GST_MESSAGE_TYPE(message)) {
    case GST_MESSAGE_ERROR:
    case GST_MESSAGE_WARNING:
      gchar *debug;
      GError *err;
      gst_message_parse_error(message, &err, &debug);
      RCLCPP_ERROR(streamer->get_logger(), "GStreamer Error: %s", err->message);
      g_error_free(err);
      g_free(debug);
      break;
    case GST_MESSAGE_STATE_CHANGED: {
      GstState old_state, new_state, pending_state;
      gst_message_parse_state_changed(message, &old_state, &new_state,
                                      &pending_state);
      RCLCPP_INFO(streamer->get_logger(), "State change: %s -> %s",
                  gst_element_state_get_name(old_state),
                  gst_element_state_get_name(new_state));
    } break;
    default:
      break;
  }
  return TRUE;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WebRTCStreamer>());
  rclcpp::shutdown();
  return 0;
}
