#include "../include/camera_streaming_qt/camera_client.h"

#include <QApplication>
#include <QDebug>
#include <vector>

#include "mainwindow.h"

CameraClient::CameraClient() : Node("camera_client_node") {}

CameraClient::~CameraClient() {}

std::vector<std::string> CameraClient::get_cameras() {
  rclcpp::Client<interfaces::srv::GetCameras>::SharedPtr client =
      this->create_client<interfaces::srv::GetCameras>("get_cameras");

  auto request = std::make_shared<interfaces::srv::GetCameras::Request>();

  rclcpp::Client<interfaces::srv::GetCameras>::SharedFuture future =
      client->async_send_request(request).future.share();

  // Wait for result, if failed, then return
  if (!wait_for_service_result<interfaces::srv::GetCameras>(future))
    return std::vector<std::string>();

  std::vector<std::string> sources = future.get()->sources;

  RCLCPP_INFO(this->get_logger(), "Received sources: ");

  for (int i = 0; i < sources.size(); i++) {
    RCLCPP_INFO(this->get_logger(), sources[i].c_str());
  }

  return sources;
}

void CameraClient::start_video(
    int num_sources, std::vector<interfaces::msg::VideoSource> sources) {
  rclcpp::Client<interfaces::srv::VideoOut>::SharedPtr client =
      this->create_client<interfaces::srv::VideoOut>("start_video");

  auto request = std::make_shared<interfaces::srv::VideoOut::Request>();
  request->num_sources = num_sources;
  request->sources = sources;

  rclcpp::Client<interfaces::srv::VideoOut>::SharedFuture future =
      client->async_send_request(request).future.share();

  // Wait for result, if failed, then return
  if (!wait_for_service_result<interfaces::srv::VideoOut>(future)) return;

  RCLCPP_INFO(get_logger(), "Start video service call succeeded.");

  open_gst_widget();
}

void CameraClient::open_gst_widget() {
  GstData gst_data;

  // Prepare the pipeline
  gst_data.pipeline = gst_pipeline_new("xvoverlay");
  gst_data.source = gst_element_factory_make("videotestsrc", NULL);
  gst_data.sink = gst_element_factory_make("ximagesink", NULL);
  gst_bin_add_many(GST_BIN(gst_data.pipeline), gst_data.source, gst_data.sink,
                   NULL);
  gst_element_link(gst_data.source, gst_data.sink);
  gst_debug_set_active(true);
  gst_debug_set_default_threshold(GST_LEVEL_WARNING);

  // Create window where the gstreamer sink output will go to
  QWidget window;
  window.resize(320, 240);
  window.show();
  gst_data.winId = window.winId();

  // For some reason X11 sink needs time before we can assign to a window (even
  // though others don't) So need to use a bus to receive message when we can
  // assign the sink to the window
  // This code was found from:
  // https://stackoverflow.com/questions/68461714/gstreamer-ximagesink-doesnt-work-when-embedded-in-gtk-window
  GstBus* bus;
  bus = gst_element_get_bus(gst_data.pipeline);
  gst_bus_set_sync_handler(
      bus, (GstBusSyncHandler)&CameraClient::bus_sync_handler, &gst_data, NULL);
  gst_object_unref(bus);
  gst_element_set_state(gst_data.pipeline, GST_STATE_PLAYING);
}

GstBusSyncReply CameraClient::bus_sync_handler(GstBus* bus, GstMessage* msg,
                                               GstData* data) {
  if (!data) return GST_BUS_PASS;

  if (!gst_is_video_overlay_prepare_window_handle_message(msg)) {
    return GST_BUS_PASS;
  }

  // Assign sink to window
  gst_video_overlay_set_window_handle(GST_VIDEO_OVERLAY(GST_MESSAGE_SRC(msg)),
                                      data->winId);

  return GST_BUS_PASS;
}

template <class T>
bool CameraClient::wait_for_service_result(
    typename rclcpp::Client<T>::SharedFuture future) {
  if (rclcpp::spin_until_future_complete(
          this->get_node_base_interface(), future,
          std::chrono::seconds(service_result_timeout_)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(get_logger(), "Service call failed.");
    return false;
  }

  return true;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto camera_client = std::make_shared<CameraClient>();

  QApplication a(argc, argv);

  gst_init(&argc, &argv);

  // Main window with all of the settings
  MainWindow* w = new MainWindow(camera_client.get(), nullptr);
  w->resize(1280, 720);
  w->show();

  a.exec();

  rclcpp::spin(camera_client);
  rclcpp::shutdown();

  return 0;
}