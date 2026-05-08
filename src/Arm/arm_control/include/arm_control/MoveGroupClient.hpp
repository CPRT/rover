#pragma once

#include <memory>
#include <string>
#include <vector>

#include "interfaces/msg/distance.hpp"
#include "interfaces/srv/get_poses.hpp"
#include "interfaces/srv/go_to_pose.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace arm_control {

class MoveGroupClient {
public:
  explicit MoveGroupClient(rclcpp::Node::SharedPtr node);
  ~MoveGroupClient();

  void stop();
  bool goToSavedPose(size_t index);
  bool goToPose(const geometry_msgs::msg::Pose &pose);
  bool goToPose(const geometry_msgs::msg::PoseStamped &pose);
  bool goToCamCoord(double u, double v);
  size_t saveCurrentPose();

private:
  std::string planning_group_;

  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Subscription<interfaces::msg::Distance>::SharedPtr depth_sub_;
  rclcpp::Service<interfaces::srv::GetPoses>::SharedPtr name_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<interfaces::srv::GoToPose>::SharedPtr go_to_pose_service_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};

  double planning_time_;
  int planning_attempts_;
  double vel_scaling_;
  double acc_scaling_;
  interfaces::msg::Distance::SharedPtr latest_distance_;

  std::vector<geometry_msgs::msg::Pose> saved_poses_;
  struct cam_intrinsics {
    double fx;
    double fy;
    int cx;
    int cy;
    std::string camera_link;
  };
  cam_intrinsics camera_info_;

  void declareParams();
  void loadParams();
  void configureMoveGroup();
  bool planAndExecute();
};
} // namespace arm_control