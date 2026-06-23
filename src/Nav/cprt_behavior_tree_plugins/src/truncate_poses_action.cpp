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

#include "cprt_behavior_tree_plugins/truncate_poses_action.hpp"

namespace nav2_behavior_tree {

TruncatePoses::TruncatePoses(const std::string &xml_tag_name,
                             const BT::NodeConfiguration &conf)
    : BT::SyncActionNode(xml_tag_name, conf) {}

BT::NodeStatus TruncatePoses::tick() {
  std::vector<geometry_msgs::msg::PoseStamped> input_goals;

  if (!getInput("input_goals", input_goals)) {
    // Missing input is a configuration error, so we fail the node
    return BT::NodeStatus::FAILURE;
  }

  int num_poses = 4;
  getInput("num_poses", num_poses);

  // Handle edge case where requested poses is zero or negative
  if (num_poses <= 0) {
    setOutput("output_goals", std::vector<geometry_msgs::msg::PoseStamped>());
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<geometry_msgs::msg::PoseStamped> output_goals;

  // If the list is already shorter than the truncation length, just pass it
  // through
  if (input_goals.size() <= static_cast<size_t>(num_poses)) {
    output_goals = input_goals;
  } else {
    // Truncate the vector to the specified length
    output_goals = std::vector<geometry_msgs::msg::PoseStamped>(
        input_goals.begin(), input_goals.begin() + num_poses);
  }

  setOutput("output_goals", output_goals);
  return BT::NodeStatus::SUCCESS;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::TruncatePoses>("TruncatePoses");
}