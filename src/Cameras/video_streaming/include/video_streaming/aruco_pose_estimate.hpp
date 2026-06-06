#pragma once

#include <map>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <unordered_map>
#include <vector>

// --- OpenCV 4.8.0 Specific Headers ---
#include <opencv2/calib3d.hpp>
#include <opencv2/objdetect.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>
#include <opencv2/opencv.hpp>
// -------------------------------------

#include <interfaces/msg/aruco_markers.hpp>

struct CameraIntrinsics {
  cv::Mat camera_matrix;
  cv::Mat dist_coeffs;
};

class ArucoPoseEstimator {
public:
  ArucoPoseEstimator(rclcpp::Node *node);
  void processImage(const cv::Mat &frame, const std::string &camera_name,
                    const rclcpp::Time &stamp);

private:
  rclcpp::Node *parent_node_;
  rclcpp::Publisher<interfaces::msg::ArucoMarkers>::SharedPtr drive_pub_;
  rclcpp::Publisher<interfaces::msg::ArucoMarkers>::SharedPtr ee_pub_;

  cv::aruco::DetectorParameters detector_params_;
  cv::aruco::Dictionary dictionary_;

  double marker_length_;
  std::vector<cv::Point3f> marker_obj_points_;

  // Camera routing and calibration mapping
  std::unordered_map<std::string, std::string> camera_to_frame_id_;
  std::unordered_map<std::string, CameraIntrinsics> calibrations_;
};