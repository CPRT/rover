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

#ifndef CPRT_BEHAVIOR_TREE_PLUGINS__REMOVE_IN_COLLISION_GOALS_ACTION_HPP_
#define CPRT_BEHAVIOR_TREE_PLUGINS__REMOVE_IN_COLLISION_GOALS_ACTION_HPP_

#include <string>
#include <vector>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree {

class RemoveInCollisionGoals : public BT::ActionNodeBase {

public:
  RemoveInCollisionGoals(const std::string &xml_tag_name,
                         const BT::NodeConfiguration &conf);

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
                "input_goals", "Goals to filter"),
            BT::InputPort<double>(
                "cost_threshold", 98.0,
                "Cost threshold (0-100 scale). 99+ is inscribed/lethal"),
            BT::InputPort<std::string>("costmap_topic",
                                       "/global_costmap/costmap",
                                       "Topic to read costmap from"),
            BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
                "output_goals", "Filtered goals")};
  }

  BT::NodeStatus tick() override;

  // Required override for ActionNodeBase. Triggered if the tree is cancelled.
  void halt() override {
    // No cleanup needed here
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string costmap_topic_;
};

} // namespace nav2_behavior_tree

#endif // CPRT_BEHAVIOR_TREE_PLUGINS__REMOVE_IN_COLLISION_GOALS_ACTION_HPP_