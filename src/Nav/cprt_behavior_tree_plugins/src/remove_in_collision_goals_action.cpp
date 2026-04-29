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

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace nav2_behavior_tree {

// --- Persistent Shared State ---
struct CostmapSubscriptionState {
  nav_msgs::msg::OccupancyGrid costmap;
  bool has_costmap{false};
  std::mutex mutex;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
      update_sub;
};

// Static map to keep subscriptions alive independently of the BT node's
// lifecycle
static std::unordered_map<std::string,
                          std::shared_ptr<CostmapSubscriptionState>>
    g_costmap_states;
static std::mutex g_state_mutex;
// -------------------------------

RemoveInCollisionGoals::RemoveInCollisionGoals(
    const std::string &xml_tag_name, const BT::NodeConfiguration &conf)
    : BT::ActionNodeBase(xml_tag_name, conf) {

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  getInput("costmap_topic", costmap_topic_);

  std::lock_guard<std::mutex> lock(g_state_mutex);

  // If this is the first time the BT is created for this topic, set up the
  // persistent subscriptions
  if (g_costmap_states.find(costmap_topic_) == g_costmap_states.end()) {
    auto state = std::make_shared<CostmapSubscriptionState>();

    // Base costmap subscription
    state->sub = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic_,
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        [state](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
          std::lock_guard<std::mutex> lock(state->mutex);
          state->costmap = *msg;
          state->has_costmap = true;
        });

    // Update costmap subscription
    std::string update_topic = costmap_topic_ + "_updates";
    state->update_sub =
        node_->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
            update_topic, rclcpp::SystemDefaultsQoS(),
            [state, node_logger = node_->get_logger()](
                const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
              std::lock_guard<std::mutex> lock(state->mutex);
              if (!state->has_costmap)
                return;

              if (msg->x + msg->width > state->costmap.info.width ||
                  msg->y + msg->height > state->costmap.info.height) {
                RCLCPP_WARN(
                    node_logger,
                    "Costmap update bounds exceed base costmap limits.");
                return;
              }

              uint32_t data_index = 0;
              for (uint32_t j = 0; j < msg->height; ++j) {
                uint32_t dest_index =
                    (msg->y + j) * state->costmap.info.width + msg->x;
                for (uint32_t i = 0; i < msg->width; ++i) {
                  state->costmap.data[dest_index + i] = msg->data[data_index++];
                }
              }
            });

    g_costmap_states[costmap_topic_] = state;
  }
}

void RemoveInCollisionGoals::halt() {
  // Required override for BT::ActionNodeBase.
  // Triggered if the behavior tree engine halts or cancels this node.
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

  // Retrieve the persistent state
  std::shared_ptr<CostmapSubscriptionState> state;
  {
    std::lock_guard<std::mutex> lock(g_state_mutex);
    state = g_costmap_states[costmap_topic_];
  }

  nav_msgs::msg::OccupancyGrid current_costmap;

  // Safely grab the active costmap for this tick, or yield if it hasn't arrived
  {
    std::lock_guard<std::mutex> lock(state->mutex);

    if (!state->has_costmap) {
      // Yield execution back to the ROS 2 executor so the costmap callback can
      // fire
      RCLCPP_INFO_THROTTLE(
          node_->get_logger(), *node_->get_clock(), 1000,
          "RemoveInCollisionGoals: Waiting for initial global costmap...");
      return BT::NodeStatus::RUNNING;
    }

    current_costmap = state->costmap;
  }

  std::vector<geometry_msgs::msg::PoseStamped> output_goals;
  const auto &info = current_costmap.info;

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
      const int8_t cost = current_costmap.data[index];

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