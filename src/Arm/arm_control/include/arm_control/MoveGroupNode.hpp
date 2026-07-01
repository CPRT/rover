#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/msg/distance.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "interfaces/msg/move_group_status.hpp"
#include "interfaces/srv/get_named_targets.hpp"
#include "interfaces/srv/go_to_cam_coord.hpp"
#include "interfaces/srv/go_to_named_pose.hpp"
#include "interfaces/srv/go_to_pose.hpp"
#include "interfaces/srv/save_current_pose.hpp"

namespace arm_control {

class MoveGroupNode : public rclcpp::Node {
public:
  explicit MoveGroupNode(const rclcpp::NodeOptions &options);
  ~MoveGroupNode() override;

private:
  struct CameraInfo {
    double fx;
    double fy;
    double cx;
    double cy;
    std::string camera_link;
  };

  void initialize();
  void declareParams();
  void loadParams();
  void configureMoveGroup();

  void publishStatus(uint8_t status, const std::string &status_text,
                     const std::string &active_target = "");

  bool planAndExecute(std::string &message,
                      const std::string &active_target = "");

  bool goToPose(const geometry_msgs::msg::PoseStamped &pose,
                std::string &message, const std::string &active_target = "");

  bool goToNamedPose(const std::string &name, std::string &message);

  bool goToCamCoord(double u, double v, std::string &message);

  void saveCurrentPoseCallback(
      const std::shared_ptr<interfaces::srv::SaveCurrentPose::Request> request,
      std::shared_ptr<interfaces::srv::SaveCurrentPose::Response> response);

  void getNamedTargetsCallback(
      const std::shared_ptr<interfaces::srv::GetNamedTargets::Request> request,
      std::shared_ptr<interfaces::srv::GetNamedTargets::Response> response);

  void goToPoseCallback(
      const std::shared_ptr<interfaces::srv::GoToPose::Request> request,
      std::shared_ptr<interfaces::srv::GoToPose::Response> response);

  void goToNamedPoseCallback(
      const std::shared_ptr<interfaces::srv::GoToNamedPose::Request> request,
      std::shared_ptr<interfaces::srv::GoToNamedPose::Response> response);

  void goToCamCoordCallback(
      const std::shared_ptr<interfaces::srv::GoToCamCoord::Request> request,
      std::shared_ptr<interfaces::srv::GoToCamCoord::Response> response);

  double planning_time_;
  int planning_attempts_;
  double vel_scaling_;
  double acc_scaling_;
  double max_depth_age_;
  double offset_to_fingers_;
  std::string planning_group_;
  std::string planning_frame_;
  CameraInfo camera_info_;

  std::atomic_bool initialized_;
  std::atomic_uint8_t current_status_;

  std::mutex command_mutex_;
  std::mutex depth_mutex_;
  std::mutex saved_poses_mutex_;

  std::map<std::string, geometry_msgs::msg::PoseStamped> saved_poses_;
  interfaces::msg::Distance::SharedPtr latest_distance_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::CallbackGroup::SharedPtr service_callback_group_;

  rclcpp::Subscription<interfaces::msg::Distance>::SharedPtr depth_sub_;

  rclcpp::Publisher<interfaces::msg::MoveGroupStatus>::SharedPtr status_pub_;

  rclcpp::Service<interfaces::srv::SaveCurrentPose>::SharedPtr
      save_current_pose_service_;

  rclcpp::Service<interfaces::srv::GetNamedTargets>::SharedPtr
      get_named_targets_service_;

  rclcpp::Service<interfaces::srv::GoToPose>::SharedPtr go_to_pose_service_;

  rclcpp::Service<interfaces::srv::GoToNamedPose>::SharedPtr
      go_to_named_pose_service_;

  rclcpp::Service<interfaces::srv::GoToCamCoord>::SharedPtr
      go_to_cam_coord_service_;

  rclcpp::TimerBase::SharedPtr initialization_timer_;
};

} // namespace arm_control