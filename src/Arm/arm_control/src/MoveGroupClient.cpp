#include "arm_control/MoveGroupClient.hpp"
#include <memory>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace arm_control {
MoveGroupClient::MoveGroupClient(rclcpp::Node::SharedPtr node)
    : planning_time_(5.0), planning_attempts_(10), vel_scaling_(0.2),
      acc_scaling_(0.2), node_(node) {
  declareParams();
  loadParams();
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  move_group_ =
      std::make_unique<moveit::planning_interface::MoveGroupInterface>(
          node_, planning_group_);

  depth_sub_ = node->create_subscription<interfaces::msg::Distance>(
      "/eef_distance", 10,
      [this](const interfaces::msg::Distance::SharedPtr msg) {
        latest_distance_ = msg;
      });

  configureMoveGroup();

  RCLCPP_INFO(node_->get_logger(),
              "MoveGroupClient ready for planning group %s",
              planning_group_.c_str());
}

MoveGroupClient::~MoveGroupClient() = default;

void MoveGroupClient::declareParams() {
  node_->declare_parameter<double>("planning_time", 5.0);
  node_->declare_parameter<std::string>("planning_group", "arm");
  node_->declare_parameter<int>("planning_attempts", 10);
  node_->declare_parameter<double>("vel_scaling", 0.2);
  node_->declare_parameter<double>("acc_scaling", 0.2);
  node_->declare_parameter<int>("cx", 960);
  node_->declare_parameter<int>("cy", 540);
  node_->declare_parameter<double>("fx", 540);
  node_->declare_parameter<double>("fy", 540);
  node_->declare_parameter<std::string>("camera_link", "EndEffector");
}

void MoveGroupClient::loadParams() {
  planning_time_ = node_->get_parameter("planning_time").as_double();
  planning_attempts_ = node_->get_parameter("planning_attempts").as_int();
  planning_group_ = node_->get_parameter("planning_group").as_string();
  vel_scaling_ = node_->get_parameter("vel_scaling").as_double();
  acc_scaling_ = node_->get_parameter("acc_scaling").as_double();
  camera_info_.fx = node_->get_parameter("fx").as_double();
  camera_info_.fy = node_->get_parameter("fy").as_double();
  camera_info_.cx = node_->get_parameter("cx").as_int();
  camera_info_.cy = node_->get_parameter("cy").as_int();
  camera_info_.camera_link = node_->get_parameter("camera_link").as_string();
}

void MoveGroupClient::configureMoveGroup() {
  move_group_->setPlanningTime(planning_time_);
  move_group_->setNumPlanningAttempts(planning_attempts_);
  move_group_->setMaxVelocityScalingFactor(vel_scaling_);
  move_group_->setMaxAccelerationScalingFactor(acc_scaling_);

  RCLCPP_INFO(node_->get_logger(), "Planning frame: %s",
              move_group_->getPlanningFrame().c_str());
  RCLCPP_INFO(node_->get_logger(), "EEF link: %s",
              move_group_->getEndEffectorLink().c_str());
}

void MoveGroupClient::stop() {
  RCLCPP_INFO(node_->get_logger(), "MoveGroupClient stopping");
  move_group_->stop();
}

bool MoveGroupClient::planAndExecute() {
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  auto result = move_group_->plan(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(node_->get_logger(), "Planning failed");
    return false;
  }

  result = move_group_->asyncExecute(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_WARN(node_->get_logger(), "Async execution start failed");
    return false;
  }

  return true;
}

bool MoveGroupClient::goToPose(const geometry_msgs::msg::Pose &pose) {
  RCLCPP_INFO(node_->get_logger(),
              "MoveGroupClient moving to pose x: %f y: %f z: %f",
              pose.position.x, pose.position.y, pose.position.z);
  move_group_->clearPoseTargets();
  move_group_->setStartStateToCurrentState();
  move_group_->setPoseTarget(pose);
  return planAndExecute();
}

bool MoveGroupClient::goToPose(const geometry_msgs::msg::PoseStamped &pose) {
  RCLCPP_INFO(node_->get_logger(),
              "MoveGroupClient moving to pose x: %f y: %f z: %f in frame: %s",
              pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
              pose.header.frame_id.c_str());
  move_group_->clearPoseTargets();
  move_group_->setStartStateToCurrentState();
  move_group_->setPoseTarget(pose);
  return planAndExecute();
}

size_t MoveGroupClient::saveCurrentPose() {
  geometry_msgs::msg::Pose current_pose = move_group_->getCurrentPose().pose;
  saved_poses_.push_back(current_pose);
  return saved_poses_.size() - 1;
}

bool MoveGroupClient::goToSavedPose(size_t index) {
  if (index >= saved_poses_.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Saved pose index %zu out of range (size=%zu)", index,
                 saved_poses_.size());
    return false;
  }

  return goToPose(saved_poses_[index]);
}

bool MoveGroupClient::goToCamCoord(double u, double v) {
  auto now = node_->get_clock()->now();
  constexpr double offset_to_fingers = 0.18;

  if (!latest_distance_ ||
      latest_distance_->status != interfaces::msg::Distance::STATUS_OK ||
      (now - latest_distance_->header.stamp).seconds() > 1.1) {
    RCLCPP_ERROR(node_->get_logger(), "Waiting on depth");
    return false;
  }

  double Z = latest_distance_->distance;
  double X = (u - camera_info_.cx) * Z / camera_info_.fx;
  double Y = (v - camera_info_.cy) * Z / camera_info_.fy;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = camera_info_.camera_link;

  pose.pose.position.x = Y;
  pose.pose.position.y = -X;
  pose.pose.position.z = 0;
  // If target is at center of camera, move staight forward by half the distance
  // to fingers + 1cm
  if (std::abs(u - camera_info_.cx) < 1e-6 &&
      std::abs(v - camera_info_.cy) < 1e-6) {
    pose.pose.position.z = (Z - offset_to_fingers) / 2.0 + 0.01;
  }

  pose.pose.orientation.x = 0;
  pose.pose.orientation.y = 0;
  pose.pose.orientation.z = 0;
  pose.pose.orientation.w = 1;

  geometry_msgs::msg::PoseStamped transformed;
  RCLCPP_INFO(node_->get_logger(),
              "Transforming pose from camera frame %s to base_link frame",
              camera_info_.camera_link.c_str());
  try {
    tf_buffer_->transform(pose, transformed, "base_link");
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(node_->get_logger(), "Transform failed: %s", ex.what());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(),
              "Moving to cam coord u: %f v: %f -> local x: %f local y: %f "
              "local z: %f -> x: %f, y: %f, z: %f",
              u, v, pose.pose.position.x, pose.pose.position.y,
              pose.pose.position.z, transformed.pose.position.x,
              transformed.pose.position.y, transformed.pose.position.z);

  return goToPose(transformed);
}
} // namespace arm_control
