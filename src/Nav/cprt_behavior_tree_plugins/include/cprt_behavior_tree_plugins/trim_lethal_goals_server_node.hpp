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

#ifndef CPRT_BEHAVIOR_TREE_PLUGINS__TRIM_LETHAL_GOALS_SERVER_NODE_HPP_
#define CPRT_BEHAVIOR_TREE_PLUGINS__TRIM_LETHAL_GOALS_SERVER_NODE_HPP_

#include <mutex>
#include <string>
#include <vector>

#include "interfaces/srv/filter_goals.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cprt_nav {

class TrimLethalGoalsServerNode : public rclcpp::Node {
public:
  explicit TrimLethalGoalsServerNode(const rclcpp::NodeOptions &options);

private:
  // Callbacks
  void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void costmapUpdateCallback(
      const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg);
  void trimGoalsCallback(
      const std::shared_ptr<interfaces::srv::FilterGoals::Request> request,
      std::shared_ptr<interfaces::srv::FilterGoals::Response> response);

  // ROS Interfaces
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
      costmap_update_sub_;
  rclcpp::Service<interfaces::srv::FilterGoals>::SharedPtr filter_service_;

  // State
  nav_msgs::msg::OccupancyGrid latest_costmap_;
  bool has_costmap_{false};
  std::mutex costmap_mutex_;
};

} // namespace cprt_nav

#endif // CPRT_BEHAVIOR_TREE_PLUGINS__TRIM_LETHAL_GOALS_SERVER_NODE_HPP_