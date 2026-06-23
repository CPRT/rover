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

#ifndef CPRT_BEHAVIOR_TREE_PLUGINS__TRUNCATE_POSES_ACTION_HPP_
#define CPRT_BEHAVIOR_TREE_PLUGINS__TRUNCATE_POSES_ACTION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace nav2_behavior_tree {

class TruncatePoses : public BT::SyncActionNode {
public:
  TruncatePoses(const std::string &xml_tag_name,
                const BT::NodeConfiguration &conf);

  // Synchronous tick - executes immediately and returns SUCCESS or FAILURE
  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
                "input_goals", "Original list of goals"),
            BT::InputPort<int>(
                "num_poses", 4,
                "Number of poses to retain from the start of the list"),
            BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
                "output_goals", "Truncated list of goals")};
  }
};

} // namespace nav2_behavior_tree

#endif // CPRT_BEHAVIOR_TREE_PLUGINS__TRUNCATE_POSES_ACTION_HPP_