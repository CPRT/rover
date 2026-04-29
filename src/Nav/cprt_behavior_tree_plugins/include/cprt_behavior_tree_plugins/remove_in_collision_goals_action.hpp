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

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "interfaces/srv/filter_goals.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree {

// Inherit directly from BtServiceNode, passing your service type
class RemoveInCollisionGoals
    : public BtServiceNode<interfaces::srv::FilterGoals> {

public:
  RemoveInCollisionGoals(const std::string &xml_tag_name,
                         const BT::NodeConfiguration &conf);

  // Called immediately when the BT ticks the node
  void on_tick() override;

  // Called when the service server replies
  BT::NodeStatus on_completion(
      std::shared_ptr<interfaces::srv::FilterGoals::Response> response)
      override;

  static BT::PortsList providedPorts() {
    // providedBasicPorts automatically injects "server_name" and
    // "server_timeout" ports
    return providedBasicPorts(
        {BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
             "input_goals", "Goals to filter"),
         BT::InputPort<double>(
             "cost_threshold", 98.0,
             "Cost threshold (0-100 scale). 99+ is inscribed/lethal"),
         BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>(
             "output_goals", "Filtered goals")});
  }
};

} // namespace nav2_behavior_tree

#endif // CPRT_BEHAVIOR_TREE_PLUGINS__REMOVE_IN_COLLISION_GOALS_ACTION_HPP_