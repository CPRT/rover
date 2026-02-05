#ifndef MOVEGROUP_CONTROLLER_HPP
#define MOVEGROUP_CONTROLLER_HPP
#include "<moveit/move_group_interface/move_group_interface.h>"
#include "action/move_to_pose.action"
#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MoveGroupController : public rclcpp::Node {
public:
  using MoveToPose = action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ClientGoalHandle<MoveToPose>;

  explicit MoveGroupController(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const MoveToPose::Goal> goal);
  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  void execute(const std::shared_ptr<GoalHandleMoveToPose> goal_handle);
  rclcpp_action::Server<MoveToPose>::SharedPtr action_server_;
  void handle_move(geometry_msgs::msg::Pose target_pose);
  const std::string PLANNING_GROUP = "rover_arm";
  moveit::planning_interface::MoveGroupInterface move_group_;

  void executeTrajectory(moveit_msgs::msg::RobotTrajectory &traj,
                         moveit::planning_interface::MoveGroupInterfacePtr mgi);
}