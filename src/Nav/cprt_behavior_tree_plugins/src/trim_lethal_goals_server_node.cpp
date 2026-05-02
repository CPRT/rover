#include "cprt_behavior_tree_plugins/trim_lethal_goals_server_node.hpp"

namespace cprt_nav {

TrimLethalGoalsServerNode::TrimLethalGoalsServerNode(
    const rclcpp::NodeOptions &options)
    : Node("trim_lethal_goals_server", options) {
  // Declare Parameters
  std::string costmap_topic =
      this->declare_parameter("costmap_topic", "/global_costmap/costmap");
  std::string service_name =
      this->declare_parameter("service_name", "trim_lethal_goals");

  RCLCPP_INFO(this->get_logger(),
              "Starting Trim Lethal Goals Server. Topic: %s, Service: %s",
              costmap_topic.c_str(), service_name.c_str());

  // 1. Subscribe to Base Costmap
  costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      costmap_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&TrimLethalGoalsServerNode::costmapCallback, this,
                std::placeholders::_1));

  // 2. Subscribe to Costmap Updates
  std::string update_topic = costmap_topic + "_updates";
  costmap_update_sub_ =
      this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
          update_topic, rclcpp::SystemDefaultsQoS(),
          std::bind(&TrimLethalGoalsServerNode::costmapUpdateCallback, this,
                    std::placeholders::_1));

  // 3. Create the Service Server
  filter_service_ = this->create_service<interfaces::srv::FilterGoals>(
      service_name,
      std::bind(&TrimLethalGoalsServerNode::trimGoalsCallback, this,
                std::placeholders::_1, std::placeholders::_2));
}

void TrimLethalGoalsServerNode::costmapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);
  latest_costmap_ = *msg;
  has_costmap_ = true;
}

void TrimLethalGoalsServerNode::costmapUpdateCallback(
    const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);

  if (!has_costmap_) {
    return;
  }

  if (msg->x + msg->width > latest_costmap_.info.width ||
      msg->y + msg->height > latest_costmap_.info.height) {
    RCLCPP_WARN(this->get_logger(),
                "Costmap update bounds exceed base costmap limits.");
    return;
  }

  uint32_t data_index = 0;
  for (uint32_t j = 0; j < msg->height; ++j) {
    uint32_t dest_index = (msg->y + j) * latest_costmap_.info.width + msg->x;
    for (uint32_t i = 0; i < msg->width; ++i) {
      latest_costmap_.data[dest_index + i] = msg->data[data_index++];
    }
  }
}

void TrimLethalGoalsServerNode::trimGoalsCallback(
    const std::shared_ptr<interfaces::srv::FilterGoals::Request> request,
    std::shared_ptr<interfaces::srv::FilterGoals::Response> response) {
  std::lock_guard<std::mutex> lock(costmap_mutex_);

  if (!has_costmap_) {
    RCLCPP_WARN(this->get_logger(),
                "Trim requested, but no costmap received yet!");
    response->output_goals = request->input_goals;
    response->success = false;
    return;
  }

  const auto &info = latest_costmap_.info;
  int threshold = static_cast<int>(request->cost_threshold);

  if (threshold > 100) {
    RCLCPP_WARN(this->get_logger(),
                "Received cost_threshold > 100. Capping to standard ROS scale max of 100.");
    threshold = 100;
  } else if (threshold < 0) {
    RCLCPP_WARN(this->get_logger(),
                "Received cost_threshold < 0. Capping to 0.");
    threshold = 0;
  }

  const int8_t validated_threshold = static_cast<int8_t>(threshold);

  for (const auto &goal : request->input_goals) {
    const double mx = goal.pose.position.x;
    const double my = goal.pose.position.y;

    const int gx =
        static_cast<int>((mx - info.origin.position.x) / info.resolution);
    const int gy =
        static_cast<int>((my - info.origin.position.y) / info.resolution);

    if (gx >= 0 && gx < static_cast<int>(info.width) && gy >= 0 &&
        gy < static_cast<int>(info.height)) {
      const int index = gy * static_cast<int>(info.width) + gx;
      const int8_t cost = latest_costmap_.data[index];

      if (cost == -1 || cost < validated_threshold) {
        response->output_goals.push_back(goal);
      }
    } else {
      response->output_goals.push_back(goal);
    }
  }

  response->success = true;
}

} // namespace cprt_nav

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(cprt_nav::TrimLethalGoalsServerNode)