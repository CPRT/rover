/**
 * @file camera_client.h
 * @brief Header file for the CameraClient class
 * @author Aria Wong
 *
 * This file contains the declaration of the CameraClient class. It also
 * contains the main function which is used to initialize the ROS2 node, Qt and
 * gstreamer. The service name is camera_client_node.
 */

#ifndef CAMERA_CLIENT_H
#define CAMERA_CLIENT_H

#include <gst/gst.h>
#include <gst/video/videooverlay.h>

#include <QWidget>
#include <interfaces/msg/video_source.hpp>
#include <interfaces/srv/get_cameras.hpp>
#include <interfaces/srv/video_out.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

/**
 * @class CameraClient
 * @brief A class that contains functions to get camera source names and start
 * the video using ROS2 services. It also contains the gstreamer code which is
 * used to display the video feeds.
 */
class CameraClient : public rclcpp::Node {
 public:
  typedef struct GstData {
    GstElement* pipeline;
    GstElement* source;
    GstElement* convert;
    GstElement* sink;
    WId winId;
  } GstData;

  CameraClient();
  ~CameraClient();

  /**
   * @brief Gets camera source names from the get_cameras service
   */
  std::vector<std::string> get_cameras();

  /**
   * @brief Submits a preset to the ROS2 service and then begins video
   */
  void start_video(int num_sources,
                   std::vector<interfaces::msg::VideoSource> sources);

 private:
  /**
   * @brief Creates gstreamer pipeline and widget to hold the gstreamer sink
   * data.
   */
  void open_gst_widget();

  /**
   * @brief Links the gstreamer sink to the widget.
   */
  GstBusSyncReply bus_sync_handler(GstBus* bus, GstMessage* message,
                                   GstData* data);

  /**
   * @brief Waits for the result from the service. If no result after
   * service_result_timeout_ seconds, then stop.
   */
  template <class T>
  bool wait_for_service_result(typename rclcpp::Client<T>::SharedFuture future);

  // How long to wait before timing out when waiting for a service result (in
  // seconds)
  const int service_result_timeout_ = 1;
};

#endif
