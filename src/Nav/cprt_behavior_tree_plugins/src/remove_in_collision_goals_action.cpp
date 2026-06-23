// Copyright (c) 2024 Angsa Robotics
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cprt_behavior_tree_plugins/remove_in_collision_goals_action.hpp"

namespace nav2_behavior_tree {

RemoveInCollisionGoals::RemoveInCollisionGoals(
    const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
    : BtServiceNode<interfaces::srv::FilterGoals>(xml_tag_name, conf) {}

void RemoveInCollisionGoals::on_tick() {
  std::vector<geometry_msgs::msg::PoseStamped> input_goals;
  if (!getInput("input_goals", input_goals)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RemoveInCollisionGoals: input_goals missing");
  }

  double cost_threshold = 98.0;
  getInput("cost_threshold", cost_threshold);

  // Populate the automatically managed request_ pointer
  request_->input_goals = input_goals;
  request_->cost_threshold = static_cast<float>(cost_threshold);
}

BT::NodeStatus RemoveInCollisionGoals::on_completion(
    std::shared_ptr<interfaces::srv::FilterGoals::Response> response) {

  if (!response->success) {
    RCLCPP_WARN(node_->get_logger(),
                "RemoveInCollisionGoals: Server failed to filter goals (likely "
                "waiting for costmap). Failing node.");
    return BT::NodeStatus::FAILURE;
  }

  // If all goals were inside obstacles, the path is blocked. Better to fail
  // early!
  if (response->output_goals.empty() && !request_->input_goals.empty()) {
    RCLCPP_WARN(node_->get_logger(),
                "RemoveInCollisionGoals: All goals were removed! (All in "
                "collision). Failing node.");
    return BT::NodeStatus::FAILURE;
  }

  setOutput("output_goals", response->output_goals);
  return BT::NodeStatus::SUCCESS;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::RemoveInCollisionGoals>(
      "RemoveInCollisionGoals");
}