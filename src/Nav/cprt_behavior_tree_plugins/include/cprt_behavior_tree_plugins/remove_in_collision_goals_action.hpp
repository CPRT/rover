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

#include <memory>
#include <string>
#include <vector>

#include "nav_msgs/msg/goals.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav2_behavior_tree/json_utils.hpp"
#include "nav2_msgs/msg/waypoint_status.hpp"
#include "nav2_msgs/srv/get_costs.hpp"

namespace cprt_behavior_tree_plugins
{

class RemoveInCollisionGoals : public nav2_behavior_tree::BtServiceNode<nav2_msgs::srv::GetCosts>
{
public:
  RemoveInCollisionGoals(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  void on_tick() override;

  BT::NodeStatus on_completion(std::shared_ptr<nav2_msgs::srv::GetCosts::Response> response) override;

  static BT::PortsList providedPorts()
  {
    // Register JSON definitions for the types used in ports.
    BT::RegisterJsonDefinition<nav_msgs::msg::Goals>();
    BT::RegisterJsonDefinition<nav2_msgs::msg::WaypointStatus>();
    BT::RegisterJsonDefinition<std::vector<nav2_msgs::msg::WaypointStatus>>();

    return providedBasicPorts(
      {
        BT::InputPort<nav_msgs::msg::Goals>(
          "input_goals",
          "Original goals to remove from"),
        BT::InputPort<double>(
          "cost_threshold", 254.0,
          "Cost threshold for considering a goal in collision"),
        BT::InputPort<bool>("use_footprint", true, "Whether to use footprint cost"),
        BT::InputPort<bool>(
          "consider_unknown_as_obstacle", false,
          "Whether to consider unknown cost as obstacle"),
        BT::OutputPort<nav_msgs::msg::Goals>(
          "output_goals",
          "Goals with in-collision goals removed"),
        BT::InputPort<std::vector<nav2_msgs::msg::WaypointStatus>>(
          "input_waypoint_statuses",
          "Original waypoint_statuses to mark waypoint status from"),
        BT::OutputPort<std::vector<nav2_msgs::msg::WaypointStatus>>(
          "output_waypoint_statuses",
          "Waypoint_statuses with in-collision waypoints marked")
      });
  }

private:
  bool use_footprint_{true};
  bool consider_unknown_as_obstacle_{false};
  double cost_threshold_{254.0};
  nav_msgs::msg::Goals input_goals_;
};

}  // namespace cprt_behavior_tree_plugins

#endif  // CPRT_BEHAVIOR_TREE_PLUGINS__REMOVE_IN_COLLISION_GOALS_ACTION_HPP_
