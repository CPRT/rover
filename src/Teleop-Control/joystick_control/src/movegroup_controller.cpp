#include "movegroup_controller.hpp"
#include <thread>

// Use placeholders for cleaner bind calls
using namespace std::placeholders;

MoveGroupController::MoveGroupController(const rclcpp::NodeOptions &options)
    : Node("movegroup_controller", options) {

  declare_parameters();
  load_parameters();
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
  RCLCPP_INFO(this->get_logger(), "Received goal request for ID: %d",
              goal->pose_id);
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
  int dest_index = goal->pose_id;

  destination.position.x = positions[dest_index][0];
  destination.position.y = positions[dest_index][1];
  destination.position.z = positions[dest_index][2];
  destination.orientation.x = positions[dest_index][3];
  destination.orientation.y = positions[dest_index][4];
  destination.orientation.z = positions[dest_index][5];
  destination.orientation.w = positions[dest_index][6];

  handle_move(destination);

  for (int i = 0; i <= 100; ++i) {
    if (goal_handle->is_canceling()) {
      result->success = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // NOTE: Fix this feedback
    // feedback->current_positions =

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

  // Use -> because move_group_ is now a shared_ptr
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

void MoveGroupController::declare_parameters() {
  this->declare_parameter<std::vector<double>>(
      "pose1", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
}

void MoveGroupController::load_parameters() {
  positions.push_back(this->get_parameter("pose1").as_double_array());
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveGroupController>();

  // Use this instead of rclcpp::spin(node)
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}