#include "tests/test_camera_node.h"

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <string>

TestCameraNode::TestCameraNode() : Node("test_camera_node") {
  get_cameras_service_ = this->create_service<interfaces::srv::GetCameras>(
      "get_cameras", std::bind(&TestCameraNode::get_cameras, this,
                               std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service ready: get_cameras");

  start_video_service_ = this->create_service<interfaces::srv::VideoOut>(
      "start_video", std::bind(&TestCameraNode::start_video, this,
                               std::placeholders::_1, std::placeholders::_2));
  RCLCPP_INFO(this->get_logger(), "Service ready: start_video");
}

TestCameraNode::~TestCameraNode() {}

void TestCameraNode::get_cameras(
    const std::shared_ptr<interfaces::srv::GetCameras::Request> request,
    std::shared_ptr<interfaces::srv::GetCameras::Response> response) {
  std::vector<std::string> names{"test source 1", "test source 2",
                                 "test source 3"};

  for (int i = 0; i < names.size(); i++) {
    response->sources.push_back(names[i]);
  }

  RCLCPP_INFO(this->get_logger(), "Sending response with sources: ");

  for (int i = 0; i < response->sources.size(); i++) {
    RCLCPP_INFO(this->get_logger(), response->sources[i].c_str());
  }
}

void TestCameraNode::start_video(
    const std::shared_ptr<interfaces::srv::VideoOut::Request> request,
    std::shared_ptr<interfaces::srv::VideoOut::Response> response) {
  RCLCPP_INFO(this->get_logger(),
              "Received request with the following video sources: ");

  for (int i = 0; i < request->sources.size(); i++) {
    interfaces::msg::VideoSource src = request->sources[i];
    RCLCPP_INFO(this->get_logger(), "Name: %s", src.name.c_str());
    RCLCPP_INFO(this->get_logger(), "Width: %d", src.width);
    RCLCPP_INFO(this->get_logger(), "Height: %d", src.height);
    RCLCPP_INFO(this->get_logger(), "X: %d", src.origin_x);
    RCLCPP_INFO(this->get_logger(), "Y: %d", src.origin_y);
  }

  response->success = true;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  TestCameraNode::SharedPtr get_camera_node =
      std::make_shared<TestCameraNode>();

  rclcpp::spin(get_camera_node);
  rclcpp::shutdown();
  return 0;
}
