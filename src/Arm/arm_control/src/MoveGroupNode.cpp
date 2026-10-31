#include "arm_control/MoveGroupNode.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <utility>

#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace arm_control {

MoveGroupNode::MoveGroupNode(const rclcpp::NodeOptions &options)
    : Node("move_group_client", options), planning_time_(5.0),
      planning_attempts_(10), vel_scaling_(0.2), acc_scaling_(0.2),
      max_depth_age_(1.1), offset_to_fingers_(0.18), initialized_(false),
      current_status_(interfaces::msg::MoveGroupStatus::IDLE) {
  declareParams();
  loadParams();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

  status_pub_ = create_publisher<interfaces::msg::MoveGroupStatus>(
      "~/status", rclcpp::QoS(1).reliable().transient_local());

  depth_sub_ = create_subscription<interfaces::msg::Distance>(
      "/eef_distance", rclcpp::QoS(1).best_effort(),
      [this](const interfaces::msg::Distance::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        latest_distance_ = msg;
      });

  service_callback_group_ =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  save_current_pose_service_ = create_service<interfaces::srv::SaveCurrentPose>(
      "~/save_current_pose",
      std::bind(&MoveGroupNode::saveCurrentPoseCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);

  get_named_targets_service_ = create_service<interfaces::srv::GetNamedTargets>(
      "~/get_named_targets",
      std::bind(&MoveGroupNode::getNamedTargetsCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);

  go_to_pose_service_ = create_service<interfaces::srv::GoToPose>(
      "~/go_to_pose",
      std::bind(&MoveGroupNode::goToPoseCallback, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);

  go_to_named_pose_service_ = create_service<interfaces::srv::GoToNamedPose>(
      "~/go_to_named_pose",
      std::bind(&MoveGroupNode::goToNamedPoseCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);

  go_to_cam_coord_service_ = create_service<interfaces::srv::GoToCamCoord>(
      "~/go_to_cam_coord",
      std::bind(&MoveGroupNode::goToCamCoordCallback, this,
                std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);

  stop_service_ = create_service<std_srvs::srv::Trigger>(
      "~/stop",
      std::bind(&MoveGroupNode::stopCallback, this, std::placeholders::_1,
                std::placeholders::_2),
      rmw_qos_profile_services_default, service_callback_group_);

  publishStatus(interfaces::msg::MoveGroupStatus::IDLE, "Initializing");

  initialization_timer_ =
      create_wall_timer(std::chrono::milliseconds(1),
                        std::bind(&MoveGroupNode::initialize, this));
}

MoveGroupNode::~MoveGroupNode() {
  if (move_group_) {
    move_group_->stop();
  }
}

void MoveGroupNode::declareParams() {
  declare_parameter<double>("planning_time", 5.0);
  declare_parameter<std::string>("planning_group", "arm");
  declare_parameter<int>("planning_attempts", 10);
  declare_parameter<double>("vel_scaling", 0.2);
  declare_parameter<double>("acc_scaling", 0.2);
  declare_parameter<int>("cx", 960);
  declare_parameter<int>("cy", 540);
  declare_parameter<double>("fx", 625.0);
  declare_parameter<double>("fy", 480.0);
  declare_parameter<std::string>("camera_link", "EndEffector");
  declare_parameter<double>("max_depth_age", 1.1);
  declare_parameter<double>("offset_to_fingers", 0.18);
}

void MoveGroupNode::loadParams() {
  planning_time_ = get_parameter("planning_time").as_double();
  planning_attempts_ = get_parameter("planning_attempts").as_int();
  planning_group_ = get_parameter("planning_group").as_string();
  vel_scaling_ = get_parameter("vel_scaling").as_double();
  acc_scaling_ = get_parameter("acc_scaling").as_double();
  camera_info_.fx = get_parameter("fx").as_double();
  camera_info_.fy = get_parameter("fy").as_double();
  camera_info_.cx = get_parameter("cx").as_int();
  camera_info_.cy = get_parameter("cy").as_int();
  camera_info_.camera_link = get_parameter("camera_link").as_string();
  max_depth_age_ = get_parameter("max_depth_age").as_double();
  offset_to_fingers_ = get_parameter("offset_to_fingers").as_double();
}

void MoveGroupNode::initialize() {
  initialization_timer_->cancel();

  try {
    move_group_ =
        std::make_unique<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), planning_group_);

    configureMoveGroup();

    initialized_ = true;

    publishStatus(interfaces::msg::MoveGroupStatus::IDLE, "Idle");

    RCLCPP_INFO(get_logger(), "MoveGroupNode ready for planning group %s",
                planning_group_.c_str());
  } catch (const std::exception &ex) {
    initialized_ = false;

    publishStatus(interfaces::msg::MoveGroupStatus::FAILED,
                  "MoveGroup initialization failed");

    RCLCPP_FATAL(get_logger(), "Failed to initialize MoveGroupInterface: %s",
                 ex.what());
  }
}

void MoveGroupNode::configureMoveGroup() {
  move_group_->setPlanningTime(planning_time_);
  move_group_->setNumPlanningAttempts(planning_attempts_);
  move_group_->setMaxVelocityScalingFactor(vel_scaling_);
  move_group_->setMaxAccelerationScalingFactor(acc_scaling_);

  planning_frame_ = move_group_->getPlanningFrame();

  RCLCPP_INFO(get_logger(), "Planning frame: %s", planning_frame_.c_str());
  RCLCPP_INFO(get_logger(), "EEF link: %s",
              move_group_->getEndEffectorLink().c_str());
}

void MoveGroupNode::publishStatus(uint8_t status,
                                  const std::string &status_text,
                                  const std::string &active_target) {
  current_status_ = status;

  interfaces::msg::MoveGroupStatus msg;
  msg.header.stamp = now();
  msg.status = status;
  msg.status_text = status_text;
  msg.active_target = active_target;

  status_pub_->publish(msg);
}

bool MoveGroupNode::planAndExecute(std::string &message,
                                   const std::string &active_target) {
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  publishStatus(interfaces::msg::MoveGroupStatus::PLANNING, "Planning",
                active_target);

  auto result = move_group_->plan(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    message = "Planning failed";

    publishStatus(interfaces::msg::MoveGroupStatus::FAILED, message,
                  active_target);

    RCLCPP_WARN(get_logger(), "%s", message.c_str());

    return false;
  }

  publishStatus(interfaces::msg::MoveGroupStatus::MOVING, "Moving",
                active_target);

  result = move_group_->execute(plan);

  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    message = "Execution failed";

    publishStatus(interfaces::msg::MoveGroupStatus::FAILED, message,
                  active_target);

    RCLCPP_WARN(get_logger(), "%s", message.c_str());

    return false;
  }

  message = "Motion completed";

  publishStatus(interfaces::msg::MoveGroupStatus::IDLE, "Idle");

  return true;
}

bool MoveGroupNode::goToPose(const geometry_msgs::msg::PoseStamped &pose,
                             std::string &message,
                             const std::string &active_target) {
  if (pose.header.frame_id.empty()) {
    message = "Pose frame_id is empty";
    return false;
  }

  RCLCPP_INFO(get_logger(), "Moving to pose x: %f y: %f z: %f in frame: %s",
              pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
              pose.header.frame_id.c_str());

  move_group_->clearPoseTargets();
  move_group_->setStartStateToCurrentState();

  if (!move_group_->setPoseTarget(pose)) {
    message = "Failed to set pose target";
    return false;
  }

  return planAndExecute(message, active_target);
}

bool MoveGroupNode::goToNamedPose(const std::string &name,
                                  std::string &message) {
  geometry_msgs::msg::PoseStamped saved_pose;
  bool saved_pose_found = false;

  {
    std::lock_guard<std::mutex> lock(saved_poses_mutex_);

    const auto saved_pose_it = saved_poses_.find(name);

    if (saved_pose_it != saved_poses_.end()) {
      saved_pose = saved_pose_it->second;
      saved_pose_found = true;
    }
  }

  if (saved_pose_found) {
    return goToPose(saved_pose, message, name);
  }

  const auto named_targets = move_group_->getNamedTargets();

  if (std::find(named_targets.begin(), named_targets.end(), name) ==
      named_targets.end()) {
    message = "Named pose '" + name + "' does not exist";
    return false;
  }

  move_group_->clearPoseTargets();
  move_group_->setStartStateToCurrentState();

  if (!move_group_->setNamedTarget(name)) {
    message = "Failed to set named pose '" + name + "'";
    return false;
  }

  return planAndExecute(message, name);
}

bool MoveGroupNode::goToCamCoord(double u, double v, std::string &message) {
  interfaces::msg::Distance::SharedPtr latest_distance;

  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_distance = latest_distance_;
  }

  const auto current_time = now();

  if (!latest_distance) {
    message = "No depth measurement received";
    return false;
  }

  if (latest_distance->status != interfaces::msg::Distance::STATUS_OK) {
    message = "Latest depth measurement is invalid";
    return false;
  }

  const rclcpp::Time depth_time(latest_distance->header.stamp);

  if ((current_time - depth_time).seconds() > max_depth_age_) {
    message = "Latest depth measurement is stale";
    return false;
  }

  const double z = latest_distance->distance;
  const double x = (u - camera_info_.cx) * z / camera_info_.fx;
  const double y = (v - camera_info_.cy) * z / camera_info_.fy;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.stamp = current_time;
  pose.header.frame_id = camera_info_.camera_link;

  pose.pose.position.x = y;
  pose.pose.position.y = -x;
  pose.pose.position.z = 0.0;

  if (std::abs(u - camera_info_.cx) < 1e-6 &&
      std::abs(v - camera_info_.cy) < 1e-6) {
    pose.pose.position.z = (z - offset_to_fingers_) / 2.0 + 0.01;
  }

  pose.pose.orientation.x = 0.0;
  pose.pose.orientation.y = 0.0;
  pose.pose.orientation.z = 0.0;
  pose.pose.orientation.w = 1.0;

  geometry_msgs::msg::PoseStamped transformed;

  RCLCPP_INFO(get_logger(),
              "Transforming pose from camera frame %s to planning frame %s",
              camera_info_.camera_link.c_str(), planning_frame_.c_str());

  try {
    transformed =
        tf_buffer_->transform(pose, planning_frame_, tf2::durationFromSec(0.5));
  } catch (const tf2::TransformException &ex) {
    message = std::string("Transform failed: ") + ex.what();

    RCLCPP_WARN(get_logger(), "%s", message.c_str());

    return false;
  }

  RCLCPP_INFO(get_logger(),
              "Moving to cam coord u: %f v: %f -> local x: %f local y: %f "
              "local z: %f -> x: %f y: %f z: %f",
              u, v, pose.pose.position.x, pose.pose.position.y,
              pose.pose.position.z, transformed.pose.position.x,
              transformed.pose.position.y, transformed.pose.position.z);

  return goToPose(transformed, message, "camera_coordinate");
}

void MoveGroupNode::saveCurrentPoseCallback(
    const std::shared_ptr<interfaces::srv::SaveCurrentPose::Request> request,
    std::shared_ptr<interfaces::srv::SaveCurrentPose::Response> response) {
  std::lock_guard<std::mutex> command_lock(command_mutex_);

  if (!initialized_ || !move_group_) {
    response->success = false;
    response->message = "MoveGroupInterface is not initialized";
    return;
  }

  if (request->name.empty()) {
    response->success = false;
    response->message = "Pose name cannot be empty";
    return;
  }

  geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

  if (current_pose.header.frame_id.empty()) {
    current_pose.header.frame_id = planning_frame_;
  }

  {
    std::lock_guard<std::mutex> poses_lock(saved_poses_mutex_);
    saved_poses_[request->name] = current_pose;
  }

  response->success = true;
  response->message = "Saved current pose as '" + request->name + "'";

  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void MoveGroupNode::getNamedTargetsCallback(
    const std::shared_ptr<interfaces::srv::GetNamedTargets::Request> request,
    std::shared_ptr<interfaces::srv::GetNamedTargets::Response> response) {
  (void)request;

  std::lock_guard<std::mutex> command_lock(command_mutex_);

  if (!initialized_ || !move_group_) {
    response->success = false;
    response->message = "MoveGroupInterface is not initialized";
    return;
  }

  response->names = move_group_->getNamedTargets();

  {
    std::lock_guard<std::mutex> poses_lock(saved_poses_mutex_);

    for (const auto &[name, pose] : saved_poses_) {
      (void)pose;

      if (std::find(response->names.begin(), response->names.end(), name) ==
          response->names.end()) {
        response->names.push_back(name);
      }
    }
  }

  std::sort(response->names.begin(), response->names.end());

  response->success = true;
  response->message = "Named poses retrieved";
}

void MoveGroupNode::goToPoseCallback(
    const std::shared_ptr<interfaces::srv::GoToPose::Request> request,
    std::shared_ptr<interfaces::srv::GoToPose::Response> response) {
  std::lock_guard<std::mutex> command_lock(command_mutex_);

  if (!initialized_ || !move_group_) {
    response->success = false;
    response->message = "MoveGroupInterface is not initialized";
    return;
  }

  response->success = goToPose(request->pose, response->message, "pose");

  if (!response->success &&
      current_status_ != interfaces::msg::MoveGroupStatus::FAILED) {
    publishStatus(interfaces::msg::MoveGroupStatus::FAILED, response->message,
                  "pose");
  }
}

void MoveGroupNode::goToNamedPoseCallback(
    const std::shared_ptr<interfaces::srv::GoToNamedPose::Request> request,
    std::shared_ptr<interfaces::srv::GoToNamedPose::Response> response) {
  std::lock_guard<std::mutex> command_lock(command_mutex_);

  if (!initialized_ || !move_group_) {
    response->success = false;
    response->message = "MoveGroupInterface is not initialized";
    return;
  }

  if (request->name.empty()) {
    response->success = false;
    response->message = "Pose name cannot be empty";

    publishStatus(interfaces::msg::MoveGroupStatus::FAILED, response->message);

    return;
  }

  response->success = goToNamedPose(request->name, response->message);

  if (!response->success &&
      current_status_ != interfaces::msg::MoveGroupStatus::FAILED) {
    publishStatus(interfaces::msg::MoveGroupStatus::FAILED, response->message,
                  request->name);
  }
}

void MoveGroupNode::goToCamCoordCallback(
    const std::shared_ptr<interfaces::srv::GoToCamCoord::Request> request,
    std::shared_ptr<interfaces::srv::GoToCamCoord::Response> response) {
  std::lock_guard<std::mutex> command_lock(command_mutex_);

  if (!initialized_ || !move_group_) {
    response->success = false;
    response->message = "MoveGroupInterface is not initialized";
    return;
  }

  response->success = goToCamCoord(request->u, request->v, response->message);

  if (!response->success &&
      current_status_ != interfaces::msg::MoveGroupStatus::FAILED) {
    publishStatus(interfaces::msg::MoveGroupStatus::FAILED, response->message,
                  "camera_coordinate");
  }
}

void MoveGroupNode::stopCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  (void)request;

  std::lock_guard<std::mutex> command_lock(command_mutex_);

  if (!initialized_ || !move_group_) {
    response->success = false;
    response->message = "MoveGroupInterface is not initialized";
    return;
  }

  move_group_->stop();

  response->success = true;
  response->message = "Motion stopped";

  publishStatus(interfaces::msg::MoveGroupStatus::IDLE, "Idle");
}

} // namespace arm_control

RCLCPP_COMPONENTS_REGISTER_NODE(arm_control::MoveGroupNode)