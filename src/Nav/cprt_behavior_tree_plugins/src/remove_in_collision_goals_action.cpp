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

#include <functional>

namespace nav2_behavior_tree {

RemoveInCollisionGoals::RemoveInCollisionGoals(
    const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
    : BT::SyncActionNode(xml_tag_name, conf) {
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string costmap_topic;
  getInput("costmap_topic", costmap_topic);

  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&RemoveInCollisionGoals::costmapCallback, this,
                std::placeholders::_1));
}

void RemoveInCollisionGoals::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = msg;
}

BT::NodeStatus RemoveInCollisionGoals::tick() {
  std::vector<geometry_msgs::msg::PoseStamped> input_goals;
  if (!getInput("input_goals", input_goals)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "RemoveInCollisionGoals: input_goals missing");
    return BT::NodeStatus::FAILURE;
  }

  double cost_threshold = 98.0;
  getInput("cost_threshold", cost_threshold);

  nav_msgs::msg::OccupancyGrid::SharedPtr current_costmap;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    current_costmap = latest_costmap_;
  }

  if (!current_costmap) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                         "RemoveInCollisionGoals: No costmap received yet.");
    setOutput("output_goals", input_goals);
    return BT::NodeStatus::SUCCESS;
  }

  std::vector<geometry_msgs::msg::PoseStamped> output_goals;
  const auto &info = current_costmap->info;

  for (const auto &goal : input_goals) {
    const double mx = goal.pose.position.x;
    const double my = goal.pose.position.y;

    const int gx =
        static_cast<int>((mx - info.origin.position.x) / info.resolution);
    const int gy =
        static_cast<int>((my - info.origin.position.y) / info.resolution);

    if (gx >= 0 && gx < static_cast<int>(info.width) && gy >= 0 &&
        gy < static_cast<int>(info.height)) {
      const int index = gy * static_cast<int>(info.width) + gx;
      const int8_t cost = current_costmap->data[index];

      if (cost == -1 || cost < static_cast<int8_t>(cost_threshold)) {
        output_goals.push_back(goal);
      }
    } else {
      output_goals.push_back(goal);
    }
  }

  setOutput("output_goals", output_goals);
  return BT::NodeStatus::SUCCESS;
}

} // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::RemoveInCollisionGoals>(
      "RemoveInCollisionGoals");
}
