#include "movegroup_controller.hpp"
#include <thread>

using namespace std::placeholders;

MoveGroupController::MoveGroupController(const rclcpp::NodeOptions &options)
    : Node("movegroup_controller", options) {

  this->action_server_ = rclcpp_action::create_server<MoveToPose>(
      this, "move_to_pose",
      std::bind(&MoveGroupController::handle_goal, this, _1, _2),
      std::bind(&MoveGroupController::handle_cancel, this, _1),
      std::bind(&MoveGroupController::handle_accepted, this, _1));
  this->move_group_ =
      std::make_shared<moveit::planning_interface::MoveGroupInterface>(
          shared_from_this(), "arm_group");

  RCLCPP_INFO(this->get_logger(), "MoveGroupController action server started");
}

rclcpp_action::GoalResponse
MoveGroupController::handle_goal(const rclcpp_action::GoalUUID &uuid,
                                 std::shared_ptr<const MoveToPose::Goal> goal) {
  (void)uuid;
  (void)goal;
  RCLCPP_INFO(this->get_logger(), "Received goal request for a new target pose");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveGroupController::handle_cancel(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  (void)goal_handle;
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveGroupController::handle_accepted(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  // Use a detached thread to handle long-running execution
  std::thread{std::bind(&MoveGroupController::execute, this, goal_handle)}
      .detach();
}

void MoveGroupController::execute(
    const std::shared_ptr<GoalHandleMoveToPose> goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<MoveToPose::Feedback>();
  auto result = std::make_shared<MoveToPose::Result>();

  handle_move(goal->pose_id);

  for (int i = 0; i <= 100; ++i) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    // NOTE: Fix this feedback
    auto current_joints = move_group_->getCurrentJointValues();
    feedback->current_position = current_joints;

    goal_handle->publish_feedback(feedback);
    rclcpp::sleep_for(std::chrono::milliseconds(50));
  }

  result->success = true;
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

void MoveGroupController::handle_move(geometry_msgs::msg::Pose target_pose) {
  RCLCPP_INFO(this->get_logger(), "Moving to pose: x:%f, y:%f, z:%f",
              target_pose.position.x, target_pose.position.y,
              target_pose.position.z);

  move_group_->setPoseTarget(target_pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  auto const ok =
      (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

  if (ok) {
    move_group_->execute(my_plan);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning failed!");
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveGroupController>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}