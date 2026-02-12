#include "movegroup_controller.hpp"

MoveGroupController::MoveGroupController(const rclcpp::NodeOptions &options)
    : Node("movegroup_controller", options) {
  this->action_server_ = rclcpp_action::create_server<MoveToPose>(
      this, "move_to_pose",
      std::bind(&MoveGroupController::handle_goal, this, _1, _2),
      std::bind(&MoveGroupController::handle_cancel, this, _1),
      std::bind(&MoveGroupController::handle_accepted, this, _1));

  this->move_group_ = moveit::planning_interface::MoveGroupInterface(
      this, this->PLANNING_GROUP);
  RCLCPP_INFO(this->get_logger(), "MoveGroupController action server started");
}

rclcpp_action::GoalResponse
MoveGroupController::handle_goal(const rclcpp::GoalUUID &uuid,
                                 std::shared_ptr<const MoveToPose::Goal> goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveGroupController::handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveGroupController::handle_accepted(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  std::thread{std::bind(&MoveGroupController::execute, this, goal_handle)}
      .detach();
}

void MoveGroupController::execute(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveToPose::Feedback>();
  auto result = std::make_shared<MoveToPose::Result>();

  // This isn't real, just a placeholder. You (yes you Seysha) need to implement
  // the movegroup go to a position
  handle_move(goal->poseID);

  for (int i = 0; i <= 100; ++i) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    feedback->progress = i;
    goal_handle->publish_feedback(feedback);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }

  result->success = true;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

void MoveGroupController::handle_move(geometry_msgs::msg::Pose target_pose) {
  // Placeholder for move group logic
  RCLCPP_INFO(this->get_logger(), "Moving to the desired pose: %f, %f, %f",
              target_pose.position.x, target_pose.position.y,
              target_pose.position.z);
  move_group_.setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group_.plan(my_plan) ==
                  moveit::planning_interface::MoveItErrorCode::SUCCESS);
  if (success) {
    move_group_->execute(my_plan);
    RCLCPP_INFO(this->get_logger(), "Move executed successfully");
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to plan the move");
  }
}
