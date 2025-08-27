#include "cprt_planner_plugins/event_horizon_planner.hpp"

#include <cmath>
#include <memory>
#include <string>

#include "nav2_util/node_utils.hpp"

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cprt_planner_plugins {

EventHorizonPlanner::EventHorizonPlanner() {}

void EventHorizonPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) {
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  logger_ = node_->get_logger();

  // Parameter initialization

  // Interpolation resolution
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".interpolation_resolution",
      rclcpp::ParameterValue(DEFAULT_INTERPOLATION_RESOLUTION));
  node_->get_parameter(name_ + ".interpolation_resolution",
                       interpolation_resolution_);

  // Horizon Distance
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".horizon_distance",
      rclcpp::ParameterValue(DEFAULT_HORIZON_DISTANCE));
  node_->get_parameter(name_ + ".horizon_distance", horizon_distance_);

  // Intermediate Tolerance
  nav2_util::declare_parameter_if_not_declared(
      node_, name_ + ".intermediate_tolerance",
      rclcpp::ParameterValue(DEFAULT_INTERMEDIATE_TOLERANCE));
  node_->get_parameter(name_ + ".intermediate_tolerance",
                       intermediate_tolerance_);

  // Create a publisher to a topic to help visualize the intermediate goal.
  new_goal_publisher_ =
      node_->create_publisher<geometry_msgs::msg::PoseStamped>(
          "intermediate_goal", 10);

  SmacPlannerHybrid::configure(parent, name, tf, costmap_ros);
}

// taken from ros-navigation/navigation2_tutorials
void EventHorizonPlanner::cleanup() {
  SmacPlannerHybrid::cleanup();
  RCLCPP_INFO(logger_, "Cleaning up plugin %s of type EventHorizonPlanner",
              name_.c_str());
}

// taken from ros-navigation/navigation2_tutorials
void EventHorizonPlanner::activate() {
  SmacPlannerHybrid::activate();
  RCLCPP_INFO(logger_, "Activating plugin %s of type EventHorizonPlanner",
              name_.c_str());
}

// taken from ros-navigation/navigation2_tutorials
void EventHorizonPlanner::deactivate() {
  SmacPlannerHybrid::deactivate();
  RCLCPP_INFO(logger_, "Deactivating plugin %s of type EventHorizonPlanner",
              name_.c_str());
}

nav_msgs::msg::Path EventHorizonPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
  RCLCPP_INFO(logger_, "Creating plan");
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(logger_,
                 "Planner will only accept start position from %s frame",
                 global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(logger_, "Planner will only accept goal position from %s frame",
                global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  // get updated goal in case it is beyond the horizon
  const geometry_msgs::msg::PoseStamped new_goal = getNewGoal(start, goal);

  if (goal != new_goal) {
    // change tolerance before planning to intermediate goal on horizon
    float original_tolerance = getTolerance();
    setTolerance(intermediate_tolerance_);

    global_path = SmacPlannerHybrid::createPlan(start, new_goal);

    setTolerance(original_tolerance);

    // get the last pose planned by the smac planner in case it was unable to
    // plan exactly to new_goal.
    geometry_msgs::msg::PoseStamped horizon_pose = global_path.poses.back();

    // create a straight line from goal on the horizon to the original goal
    int total_number_of_loop =
        std::hypot(goal.pose.position.x - horizon_pose.pose.position.x,
                   goal.pose.position.y - horizon_pose.pose.position.y) /
        interpolation_resolution_;

    if (total_number_of_loop > 0) {
      double x_increment =
          (goal.pose.position.x - horizon_pose.pose.position.x) /
          total_number_of_loop;
      double y_increment =
          (goal.pose.position.y - horizon_pose.pose.position.y) /
          total_number_of_loop;

      for (int i = 0; i < total_number_of_loop; ++i) {
        geometry_msgs::msg::PoseStamped pose;
        pose.pose.position.x = horizon_pose.pose.position.x + x_increment * i;
        pose.pose.position.y = horizon_pose.pose.position.y + y_increment * i;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = horizon_pose.pose.orientation;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        global_path.poses.push_back(pose);
      }
    }

    geometry_msgs::msg::PoseStamped goal_pose = goal;
    goal_pose.header.stamp = node_->now();
    goal_pose.header.frame_id = global_frame_;
    global_path.poses.push_back(goal_pose);
  } else {
    global_path = SmacPlannerHybrid::createPlan(start, goal);
  }

  return global_path;
}

const geometry_msgs::msg::PoseStamped EventHorizonPlanner::getNewGoal(
    const geometry_msgs::msg::PoseStamped& start,
    const geometry_msgs::msg::PoseStamped& goal) {
  geometry_msgs::msg::PoseStamped new_goal;

  const double distance_to_goal =
      std::sqrt(std::pow(goal.pose.position.x - start.pose.position.x, 2) +
                std::pow(goal.pose.position.y - start.pose.position.y, 2));

  if (distance_to_goal > horizon_distance_) {
    float angle = std::atan2(goal.pose.position.y - start.pose.position.y,
                             goal.pose.position.x - start.pose.position.x);
    new_goal.pose.position.x =
        start.pose.position.x + horizon_distance_ * std::cos(angle);
    new_goal.pose.position.y =
        start.pose.position.y + horizon_distance_ * std::sin(angle);
    new_goal.pose.position.z = 0.0;
    new_goal.pose.orientation =
        EventHorizonPlanner::EulerToQuaternion(0.0, 0.0, angle);
    new_goal.header.stamp = node_->now();
    new_goal.header.frame_id = global_frame_;
  } else {
    new_goal = goal;
  }

  // Publish the new goal
  new_goal_publisher_->publish(new_goal);

  RCLCPP_INFO(logger_, "~~~~~\nIntermediate goal: %f, %f \n~~~~~~",
              new_goal.pose.position.x, new_goal.pose.position.y);

  return new_goal;
}

bool EventHorizonPlanner::setTolerance(float value) {
  if (value >= 0) {
    _tolerance = value;
    return true;
  } else {
    return false;
  }
}

float EventHorizonPlanner::getTolerance() { return _tolerance; }

geometry_msgs::msg::Quaternion EventHorizonPlanner::EulerToQuaternion(
    float roll, float pitch, float yaw) {
  float cy = std::cos(yaw * 0.5);
  float sy = std::sin(yaw * 0.5);
  float cp = std::cos(pitch * 0.5);
  float sp = std::sin(pitch * 0.5);
  float cr = std::cos(roll * 0.5);
  float sr = std::sin(roll * 0.5);

  geometry_msgs::msg::Quaternion q;
  q.w = cy * cp * cr + sy * sp * sr;
  q.x = cy * cp * sr - sy * sp * cr;
  q.y = sy * cp * sr + cy * sp * cr;
  q.z = sy * cp * cr - cy * sp * sr;
  return q;
}

}  // namespace cprt_planner_plugins

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cprt_planner_plugins::EventHorizonPlanner,
                       nav2_core::GlobalPlanner)
