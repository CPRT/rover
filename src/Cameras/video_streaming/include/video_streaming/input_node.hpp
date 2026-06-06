#pragma once
#include "base_video_node.hpp"
#include <interfaces/srv/get_cameras.hpp>
#include <interfaces/srv/video_out.hpp>
#include <std_msgs/msg/string.hpp> // Added to fix the compiler error

class InputNode : public BaseVideoNode {
public:
  explicit InputNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~InputNode() override = default;

protected:
  enum class CameraType {
    V4l2Src = 0, /**< V4L2 source */
    TestSrc,     /**< Test source */
    NetworkSrc,  /**< Network source */
  };
  bool create_pipeline() override;
  void declare_parameters();

  /**
   * @brief Callback function to configure the video feed.
   *
   * @param request The request object containing video output parameters.
   * @param response The response object to be populated with the result.
   */
  void
  video_cb(const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
           std::shared_ptr<interfaces::srv::VideoOut::Response> response);

private:
  std::map<std::string, size_t> source_map_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_camera_pub_;
  rclcpp::Service<interfaces::srv::VideoOut>::SharedPtr video_service_;
  rclcpp::Service<interfaces::srv::GetCameras>::SharedPtr get_cam_service_;
  std::shared_ptr<interfaces::srv::VideoOut::Request> current_video_request_;
};