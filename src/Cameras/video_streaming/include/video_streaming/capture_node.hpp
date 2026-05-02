#pragma once

#include "interfaces/srv/video_capture.hpp"
#include <rclcpp/rclcpp.hpp>

class VideoCaptureNode : public rclcpp::Node {
public:
  explicit VideoCaptureNode(const rclcpp::NodeOptions &options);

private:
  rclcpp::Service<interfaces::srv::VideoCapture>::SharedPtr service_;
  std::string listen_to_;

  void handle_capture(
      const std::shared_ptr<interfaces::srv::VideoCapture::Request> request,
      std::shared_ptr<interfaces::srv::VideoCapture::Response> response);
};