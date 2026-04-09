#ifndef MOVEGROUP_CONTROLLER_HPP
#define MOVEGROUP_CONTROLLER_HPP
#include "geometry_msgs/msg/pose.hpp"
#include "interfaces/action/move_to_pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/action/get_pose.hpp"

class MoveGroupController : public rclcpp::Node {
public:
  using MoveToPose = interfaces::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;
  
  using GetPose = interfaces::action::GetPose;
  using GoalHandleGetPose = rclcpp_action::ServerGoalHandle<GetPose>;

  explicit MoveGroupController(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

void init();

private:
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const MoveToPose::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  rclcpp_action::Server<GetPose>::SharedPtr action_server_2_;
  rclcpp_action::GoalResponse
  handle_goal_get_pose(const rclcpp_action::GoalUUID &uuid,
                       std::shared_ptr<const GetPose::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel_get_pose(const std::shared_ptr<GoalHandleGetPose> goal_handle);
  void handle_accepted_get_pose(
      const std::shared_ptr<GoalHandleGetPose> goal_handle);
  void execute_get_pose(const std::shared_ptr<GoalHandleGetPose> goal_handle);
  bool handle_move(geometry_msgs::msg::Pose target_pose);
  const std::string PLANNING_GROUP = "rover_arm";
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  void executeTrajectory(moveit_msgs::msg::RobotTrajectory &traj,
                         moveit::planning_interface::MoveGroupInterfacePtr mgi);
};
#endif