#pragma once
#include "base_video_node.hpp"
#include <interfaces/srv/get_cameras.hpp>
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

  void
  video_cb(const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
           std::shared_ptr<interfaces::srv::VideoOut::Response> response);

  // Apply fixed grid layout to the mosaic compositor after pipeline creation.
  bool set_mosaic_layout();

private:
  std::map<std::string, size_t> source_map_;
  rclcpp::Service<interfaces::srv::VideoOut>::SharedPtr video_service_;
  rclcpp::Service<interfaces::srv::GetCameras>::SharedPtr get_cam_service_;
  std::shared_ptr<interfaces::srv::VideoOut::Request> current_video_request_;
};
