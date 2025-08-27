
#ifndef CPRT_PLANNER_PLUGINS_EVENT_HORIZON_PLANNER_HPP_
#define CPRT_PLANNER_PLUGINS_EVENT_HORIZON_PLANNER_HPP_
#define _USE_MATH_DEFINES

#include <limits.h>

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_smac_planner/smac_planner_hybrid.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace cprt_planner_plugins {

/**
 * @class event_horizon_planner::EventHorizonPlanner
 * @brief Create a path using another specified planner up to a specified
 * horizon before completing the rest with a straight line.
 */
class EventHorizonPlanner : public nav2_smac_planner::SmacPlannerHybrid {
 public:
  /**
   * @brief Constructor for planner::EventHorizonPlanner
   */
  EventHorizonPlanner();

  /**
   * @brief Destructor for planner::EventHorizonPlanner
   */
  ~EventHorizonPlanner() = default;

  /**
   * @brief Configure planner state machine
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param tf TF buffer
   * @param costmap_ros Costmap2DROS object of environment
   */
  void configure(
      const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent, std::string name,
      std::shared_ptr<tf2_ros::Buffer> tf,
      std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  /**
   * @brief Cleanup planner state machine
   */
  void cleanup() override;

  /**
   * @brief Activate planner state machine
   */
  void activate() override;

  /**
   * @brief Deactivate planner state machine
   */
  void deactivate() override;

  /**
   * @brief Create plan using parent smac planner function up to the horizon,
   * then a straight line to the final goal if needed.
   * @param start Start pose
   * @param goal Goal pose
   * @return Path to goal pose from start pose
   */
  nav_msgs::msg::Path createPlan(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal) override;

  /**
   * @brief Setter for the tolerance parameter.
   * @param value Value to set the tolerance to.
   * @return true if inputted value is within accepted range.
   */
  bool setTolerance(float value);

  /**
   * @brief Getter for the tolerance parameter.
   * @return Value of tolerance parameter.
   */
  float getTolerance();

 private:
  /**
   * @brief Provides a new goal pose that is on/within the horizon on the line
   * between the start and goal if needed.
   * @param start Start pose
   * @param goal Goal pose
   * @return Updated goal pose
   */
  const geometry_msgs::msg::PoseStamped getNewGoal(
      const geometry_msgs::msg::PoseStamped& start,
      const geometry_msgs::msg::PoseStamped& goal);

  /**
   * @brief Convert euler vector to a quaternion
   * @param roll
   * @param pitch
   * @param yaw
   * @return Quaternion equivalent
   */
  static geometry_msgs::msg::Quaternion EulerToQuaternion(float roll,
                                                          float pitch,
                                                          float yaw);

  // parameters

  std::shared_ptr<tf2_ros::Buffer> tf_;

  nav2_util::LifecycleNode::SharedPtr node_;

  nav2_costmap_2d::Costmap2D* costmap_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr
      new_goal_publisher_;

  std::string global_frame_, name_;

  double interpolation_resolution_;

  double const DEFAULT_INTERPOLATION_RESOLUTION = 0.1;

  double horizon_distance_;

  double const DEFAULT_HORIZON_DISTANCE = std::numeric_limits<double>::max();

  float intermediate_tolerance_;

  float const DEFAULT_INTERMEDIATE_TOLERANCE = 10.0;

  rclcpp::Logger logger_{rclcpp::get_logger("RotationShimController")};
};

}  // namespace cprt_planner_plugins

#endif