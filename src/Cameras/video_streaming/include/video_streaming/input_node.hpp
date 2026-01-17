#pragma once
#include "base_video_node.hpp"
#include <interfaces/srv/get_cameras.hpp>
#include <interfaces/srv/video_capture.hpp>
#include <interfaces/srv/video_out.hpp>

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

  /**
   * @brief Callback function to get available camera sources.
   *
   * @param request The request object (not used).
   * @param response The response object to be populated with the camera
   * sources.
   */
  void get_cameras_cb(
      const std::shared_ptr<interfaces::srv::GetCameras::Request> request,
      std::shared_ptr<interfaces::srv::GetCameras::Response> response);

private:
  std::map<std::string, size_t> source_map_;
  rclcpp::Service<interfaces::srv::VideoOut>::SharedPtr video_service_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters); 
};