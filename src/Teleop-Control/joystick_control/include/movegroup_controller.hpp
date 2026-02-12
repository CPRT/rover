#ifndef MOVEGROUP_CONTROLLER_HPP
#define MOVEGROUP_CONTROLLER_HPPsu
#include "geometry_msgs/msg/pose.hpp"
#include "interfaces/action/move_to_pose.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/robot_state/robot_state.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class MoveGroupController : public rclcpp::Node {
public:
  using MoveToPose = interfaces::action::MoveToPose;
  using GoalHandleMoveToPose = rclcpp_action::ServerGoalHandle<MoveToPose>;

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
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  void executeTrajectory(moveit_msgs::msg::RobotTrajectory &traj,
                         moveit::planning_interface::MoveGroupInterfacePtr mgi);
  geometry_msgs::msg::Pose destination;

  std::vector<std::vector<double>> positions;

  void declare_parameters();
  void load_parameters();
};
#endif