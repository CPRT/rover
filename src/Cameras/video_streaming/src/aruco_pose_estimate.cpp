#include "aruco_pose_estimate.hpp"
#include <geometry_msgs/msg/point.hpp>

ArucoPoseEstimator::ArucoPoseEstimator(rclcpp::Node *node)
    : parent_node_(node) {
  drive_pub_ = parent_node_->create_publisher<interfaces::msg::ArucoMarkers>(
      "/computer_vision/drive_nav_markers", 10);
  ee_pub_ = parent_node_->create_publisher<interfaces::msg::ArucoMarkers>(
      "/computer_vision/ee_nav_markers", 10);

  dictionary_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  detector_params_ = cv::aruco::DetectorParameters();

  marker_length_ =
      parent_node_->declare_parameter("marker_size", 0.15); // 15cm default

  float half_l = marker_length_ / 2.0f;
  marker_obj_points_ = {
      cv::Point3f(-half_l, half_l, 0), cv::Point3f(half_l, half_l, 0),
      cv::Point3f(half_l, -half_l, 0), cv::Point3f(-half_l, -half_l, 0)};

  // -------------------------------------------------------------
  // Define frame IDs map based on active camera string
  // -------------------------------------------------------------
  camera_to_frame_id_["Drive"] = "DriveCamera";
  camera_to_frame_id_["EndEffector"] = "EndEffector";

  // -------------------------------------------------------------
  // Define Intrinsic Calibrations
  // -------------------------------------------------------------

  // Drive Camera
  CameraIntrinsics drive_calib;
  drive_calib.camera_matrix = (cv::Mat_<double>(3, 3) << 1208.455865000, 0,
                               953.045426800, 0, 1201.369845000,
                               590.105458700, 0, 0, 1);
  drive_calib.dist_coeffs =
      (cv::Mat_<double>(1, 8)
           << -1.041143259e-01, -8.638070390e-03, 1.363747596e-03,
       -2.526459586e-05, 4.666582799e-02, 1.310149040e-04,
       -5.928740585e-04, 4.767364821e-02);
  calibrations_["Drive"] = drive_calib;

  // End Effector Camera  (High Distortion)
  CameraIntrinsics ee_calib;
  ee_calib.camera_matrix = (cv::Mat_<double>(3, 3) << 234.2702, 0, 346.4237, 0,
                            255.8694, 287.5284, 0, 0, 1);
  ee_calib.dist_coeffs = (cv::Mat_<double>(1, 8) << 0.6030, 0.4734, -0.0011,
                          -0.0004, -0.0708, 0.3530, 0.6553, -0.0759);
  calibrations_["EndEffector"] = ee_calib;
}

void ArucoPoseEstimator::processImage(const cv::Mat &frame,
                                      const std::string &camera_name,
                                      const rclcpp::Time &stamp) {
  try {
    // Resolve Frame ID
    std::string frame_id = camera_name; // Fallback
    if (camera_to_frame_id_.find(camera_name) != camera_to_frame_id_.end()) {
      frame_id = camera_to_frame_id_[camera_name];
    }

    // Fetch correct calibration
    if (calibrations_.find(camera_name) == calibrations_.end()) {
      RCLCPP_WARN_THROTTLE(
          parent_node_->get_logger(), *parent_node_->get_clock(), 2000,
          "No calibration found for camera: %s. Skipping Aruco.",
          camera_name.c_str());
      return;
    }

    cv::Mat cam_mat = calibrations_[camera_name].camera_matrix;
    cv::Mat dist = calibrations_[camera_name].dist_coeffs;

    cv::Mat undistorted;
    cv::undistort(frame, undistorted, cam_mat, dist);

    cv::aruco::ArucoDetector detector(dictionary_, detector_params_);
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    detector.detectMarkers(undistorted, markerCorners, markerIds,
                           rejectedCandidates);

    interfaces::msg::ArucoMarkers msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;

    if (!markerIds.empty()) {
      for (size_t i = 0; i < markerIds.size(); i++) {
        msg.marker_ids.push_back(markerIds[i]);

        cv::Vec3d rvec, tvec;
        bool success =
            cv::solvePnP(marker_obj_points_, markerCorners[i], cam_mat, dist,
                         rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

        geometry_msgs::msg::Point pt;
        if (success) {
          pt.x = tvec[0];
          pt.y = tvec[1];
          pt.z = tvec[2];
        }
        msg.points.push_back(pt);
        msg.is_moving.push_back(false);
      }
    }

    if (camera_name == "EndEffector") {
      ee_pub_->publish(msg);
    } else {
      drive_pub_->publish(msg);
    }
  } catch (const std::exception &e) {
    RCLCPP_WARN(parent_node_->get_logger(), "Aruco logic failed: %s", e.what());
  }
}