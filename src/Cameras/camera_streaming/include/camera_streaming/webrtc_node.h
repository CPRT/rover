/**
 * @file webrtc_node.h
 * @brief Header file for the WebRTCStreamer class.
 * @author Connor Needham
 *
 * This file contains the declaration of the WebRTCStreamer class, which is
 * responsible for handling video streaming using WebRTC and GStreamer.
 */

#ifndef WEBRTC_STREAMER_HPP
#define WEBRTC_STREAMER_HPP

#include <gst/gst.h>

#include <interfaces/srv/get_cameras.hpp>
#include <interfaces/srv/video_capture.hpp>
#include <interfaces/srv/video_out.hpp>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <vector>

template <typename T>
struct GstDeleter {
  void operator()(T* object) const {
    if (object) {
      gst_object_unref(object);
    }
  }
};

// Type alias for unique_ptr managing a GStreamer object
template <typename T>
using GstUniquePtr = std::unique_ptr<T, GstDeleter<T>>;

/**
 * @class WebRTCStreamer
 * @brief A class for streaming video using WebRTC and GStreamer.
 *
 * The WebRTCStreamer class provides functionality to start video streaming
 * from different camera sources, manage the GStreamer pipeline, and handle
 * video output requests.
 */
class WebRTCStreamer : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for WebRTCStreamer.
   */
  WebRTCStreamer();

  /**
   * @brief Destructor for WebRTCStreamer.
   */
  ~WebRTCStreamer();

  /**
   * @enum CameraType
   * @brief Enum representing the type of camera source.
   */
  enum class CameraType {
    V4l2Src = 0, /**< V4L2 source */
    TestSrc,     /**< Test source */
    NetworkSrc,  /**< Network source */
  };

  /**
   * @struct CameraSource
   * @brief Struct representing a camera source.
   */
  struct CameraSource {
    std::string name;
    std::string path;
    CameraType type;
    bool encoded;
    bool aruco;
  };

 private:
  /**
   * @brief Initializes the GStreamer pipeline
   */
  bool start();
  /**
   * @brief Shuts down and cleans up the gstreamer pipeline resources
   */
  void stop();

  /**
   * @brief Declares parameters for the WebRTCStreamer node.
   *
   * This function declares parameters such as web server settings, camera
   * names, and camera properties.
   */
  void declare_parameters();

  /**
   * @brief Callback function to start video streaming.
   *
   * @param request The request object containing video output parameters.
   * @param response The response object to be populated with the result.
   */
  void start_video_cb(
      const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
      std::shared_ptr<interfaces::srv::VideoOut::Response> response);

  /**
   * @brief Callback function to handle video capture requests.
   *
   * @param request The request object containing video capture parameters.
   * @param response The response object to be populated with the result.
   */
  void capture_frame(
      const std::shared_ptr<interfaces::srv::VideoCapture::Request> request,
      std::shared_ptr<interfaces::srv::VideoCapture::Response> response);

  /**
   * @brief Callback function to get available camera sources.
   *
   * @param request The request object (not used).
   * @param response The response object to be populated with the camera
   * sources.
   */
  void get_cameras(
      const std::shared_ptr<interfaces::srv::GetCameras::Request> request,
      std::shared_ptr<interfaces::srv::GetCameras::Response> response);

  /**
   * @brief Callback function to restart the video pipeline.
   *
   * @param request The request object (not used).
   * @param response The response object to be populated with the result.
   */
  void restart_pipeline(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  /**
   * @brief Creates a GStreamer source element for the given camera source.
   *
   * @param src The camera source information.
   * @return A pointer to the created GStreamer element.
   */
  GstElement* create_source(const CameraSource& src);

  /**
   * @brief Updates the GStreamer pipeline based on the video output request.
   *
   * @param request The request object containing video output parameters.
   * @return True if the pipeline was successfully updated, false otherwise.
   */
  bool update_pipeline(
      const std::shared_ptr<interfaces::srv::VideoOut::Request> request);

  /**
   * @brief Initializes the GStreamer pipeline.
   *
   * @return A pointer to the initialized GStreamer pipeline element.
   */
  GstElement* initialize_pipeline();

  /**
   * @brief Unlinks current sources from the compositor element.
   */
  void unlink_sources_from_compositor();

  /**
   * @brief Unlinks a specific pad from its peer.
   *
   * @param pad The pad to be unlinked.
   * @return True if the pad was successfully unlinked, false otherwise.
   */
  static bool unlink_pad(GstPad* pad);

  /**
   * @brief Creates a GStreamer video converter element.
   *
   * @return A pointer to the created video converter element.
   */
  GstElement* create_vid_conv();
  /**
   * @brief Adds a chain of GStreamer elements to the pipeline.
   *
   * @param chain The vector of GStreamer elements to be added.
   * @return A pointer to the last element in the chain.
   */
  GstElement* add_element_chain(const std::vector<GstElement*>& chain);
  /**
   * @brief Creates a GStreamer element of the specified type.
   *
   * @param element_type The type of the GStreamer element to create.
   * @param element_name The name of the GStreamer element (optional).
   * @return A pointer to the created GStreamer element.
   */
  GstElement* create_element(std::string element_type,
                             std::string element_name = "");

  bool web_server_; /**< Flag indicating if the web server is enabled */
  std::string web_server_path_;       /**< Path to the web server */
  std::vector<GstElement*> elements_; /**< List of GStreamer elements */
  GstUniquePtr<GstElement> pipeline_; /**< GStreamer pipeline element */
  GstElement* compositor_;            /**< GStreamer compositor element */
  rclcpp::Service<interfaces::srv::VideoOut>::SharedPtr
      start_video_service_; /**< ROS2 service for starting video output */
  rclcpp::Service<interfaces::srv::VideoCapture>::SharedPtr
      capture_service_; /**< ROS2 service for capturing frames */
  rclcpp::Service<interfaces::srv::GetCameras>::SharedPtr
      get_cams_service_; /**< ROS2 service for getting camera list */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr
      restart_service_; /**< ROS2 service for restarting the video pipeline */
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr
      marker_pub_; /**< ROS2 publisher for marker detection messages */
  GstUniquePtr<GstBus> bus_; /**< GStreamer bus for message handling */
  std::map<std::string, GstUniquePtr<GstPad>>
      source_pads_; /**< Maps camera names to compositor pads*/
  int height_;      /**< Max height of the video */
  int width_;       /**< Max width of the video */
  int framerate_;   /**< Max framerate of the video */

  /**
   * @brief Callback function for handling GStreamer bus messages.
   *
   * @param bus The GStreamer bus.
   * @param message The GStreamer message.
   * @param user_data User data passed to the callback.
   * @return True if the message was successfully handled, false otherwise.
   */
  static gboolean on_bus_message(GstBus* bus, GstMessage* message,
                                 gpointer user_data);
};

#endif  // WEBRTC_STREAMER_HPP
