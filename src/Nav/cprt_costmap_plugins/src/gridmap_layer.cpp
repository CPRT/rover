#include "cprt_costmap_plugins/gridmap_layer.hpp"

#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <chrono>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cprt_costmap_plugins {

GridmapLayer::GridmapLayer() : total_execution_time_(0.0), sample_count_(0) {
  last_log_time_ = std::chrono::steady_clock::now();
}

GridmapLayer::~GridmapLayer() {}

void GridmapLayer::onInitialize() {
  global_frame_ = layered_costmap_->getGlobalFrameID();

  getParameters();

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  map_sub_ = node->create_subscription<grid_map_msgs::msg::GridMap>(
      map_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&GridmapLayer::incomingMap, this, std::placeholders::_1));
}

void GridmapLayer::activate() { matchSize(); }

void GridmapLayer::deactivate() {}

void GridmapLayer::reset() {
  resetMaps();
  has_updated_data_ = true;
  current_ = false;
}

void GridmapLayer::matchSize() {
  nav2_costmap_2d::Costmap2D *master = layered_costmap_->getCostmap();
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
            master->getResolution(), master->getOriginX(),
            master->getOriginY());
}

void GridmapLayer::getParameters() {
  int temp_lethal_threshold = 0;
  double temp_tf_tol = 0.0;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("map_topic", rclcpp::ParameterValue("/map"));
  declareParameter("layer_name", rclcpp::ParameterValue("traversability_map"));
  declareParameter("use_interpolation", rclcpp::ParameterValue(true));
  declareParameter("enable_log", rclcpp::ParameterValue(false));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "map_topic", map_topic_);
  node->get_parameter(name_ + "." + "layer_name", layer_name_);
  node->get_parameter(name_ + "." + "use_interpolation", use_interpolation_);
  node->get_parameter(name_ + "." + "enable_log", enable_log_);

  node->get_parameter("track_unknown_space", track_unknown_space_);
  node->get_parameter("use_maximum", use_maximum_);
  node->get_parameter("lethal_cost_threshold", temp_lethal_threshold);
  node->get_parameter("trinary_costmap", trinary_costmap_);
  node->get_parameter("transform_tolerance", temp_tf_tol);

  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 255), 0);
  map_received_ = false;
  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);

  setDefaultValue(track_unknown_space_ ? NO_INFORMATION : FREE_SPACE);
}

unsigned char GridmapLayer::interpretValue(double value) {
  if (!std::isfinite(value)) {
    return track_unknown_space_ ? NO_INFORMATION : FREE_SPACE;
  }

  // Gridmap values are typically 0.0-1.0
  double scaled_value = value * 255.0;

  if (scaled_value >= lethal_threshold_) {
    return LETHAL_OBSTACLE;
  }
  if (trinary_costmap_) {
    return FREE_SPACE;
  }

  const double scale = scaled_value / lethal_threshold_;
  return static_cast<unsigned char>(scale * LETHAL_OBSTACLE);
}

void GridmapLayer::incomingMap(
    const grid_map_msgs::msg::GridMap::SharedPtr new_map) {
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  grid_map::GridMapRosConverter::fromMessage(*new_map, gridmap_in_);
  map_received_ = true;
  has_updated_data_ = true;
}

void GridmapLayer::updateBounds(double robot_x, double robot_y,
                                double robot_yaw, double *min_x, double *min_y,
                                double *max_x, double *max_y) {
  if (!map_received_) {
    map_received_in_update_bounds_ = false;
    return;
  }
  map_received_in_update_bounds_ = true;

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());

  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2,
                 robot_y - getSizeInMetersY() / 2);
  }

  useExtraBounds(min_x, min_y, max_x, max_y);

  geometry_msgs::msg::TransformStamped transform;
  if (!getTransform(transform))
    return;

  double width = gridmap_in_.getLength().x();
  double height = gridmap_in_.getLength().y();

  geometry_msgs::msg::PointStamped gm_p, cp_p;
  gm_p.point.x = gridmap_in_.getPosition().x();
  gm_p.point.y = gridmap_in_.getPosition().y();
  tf2::doTransform(gm_p, cp_p, transform);

  // 1. Calculate CURRENT bounding box
  double current_min_x = cp_p.point.x - width / 2.0;
  double current_min_y = cp_p.point.y - height / 2.0;
  double current_max_x = cp_p.point.x + width / 2.0;
  double current_max_y = cp_p.point.y + height / 2.0;

  // 2. Expand Master bounds to cover the CURRENT position
  *min_x = std::min(current_min_x, *min_x);
  *min_y = std::min(current_min_y, *min_y);
  *max_x = std::max(current_max_x, *max_x);
  *max_y = std::max(current_max_y, *max_y);

  // 3. Expand Master bounds to cover the PREVIOUS position
  // This forces Nav2 to wipe the old area if localization jumped!
  if (has_last_bounds_) {
    *min_x = std::min(last_min_x_, *min_x);
    *min_y = std::min(last_min_y_, *min_y);
    *max_x = std::max(last_max_x_, *max_x);
    *max_y = std::max(last_max_y_, *max_y);
  }

  // 4. Save current position for the NEXT cycle
  last_min_x_ = current_min_x;
  last_min_y_ = current_min_y;
  last_max_x_ = current_max_x;
  last_max_y_ = current_max_y;
  has_last_bounds_ = true;
}

void GridmapLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                               int min_i, int min_j, int max_i, int max_j) {
  auto start_time = std::chrono::steady_clock::now();

  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_ || !map_received_in_update_bounds_)
    return;

  geometry_msgs::msg::TransformStamped tf_msg;
  if (!getTransform(tf_msg))
    return;

  tf2::Transform tf2_transform;
  tf2::fromMsg(tf_msg.transform, tf2_transform);
  tf2::Transform tf2_inv = tf2_transform.inverse();

  if (!gridmap_in_.exists(layer_name_)) {
    RCLCPP_ERROR_THROTTLE(logger_, *(clock_), 5000, "Layer %s missing",
                          layer_name_.c_str());
    return;
  }

  // CREATE SAFE ZONE (Prevents Edge Interpolation Tearing)
  double gm_res = gridmap_in_.getResolution();
  double safe_half_width =
      std::max(0.0, (gridmap_in_.getLength().x() / 2.0) - (2.0 * gm_res));
  double safe_half_height =
      std::max(0.0, (gridmap_in_.getLength().y() / 2.0) - (2.0 * gm_res));
  grid_map::Position gm_center = gridmap_in_.getPosition();

  // Iterate over the requested Costmap update window
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {

      double wx, wy;
      mapToWorld(i, j, wx, wy);

      tf2::Vector3 cp(wx, wy, 0.0);
      tf2::Vector3 gp = tf2_inv * cp;
      grid_map::Position pos(gp.x(), gp.y());

      // Only attempt interpolation if we are strictly INSIDE the safe zone
      if (std::abs(pos.x() - gm_center.x()) <= safe_half_width &&
          std::abs(pos.y() - gm_center.y()) <= safe_half_height) {
        try {
          float value;
          if (use_interpolation_) {
            value = gridmap_in_.atPosition(
                layer_name_, pos, grid_map::InterpolationMethods::INTER_LINEAR);
          } else {
            value = gridmap_in_.atPosition(layer_name_, pos);
          }

          unsigned char interpreted_cost = interpretValue(value);
          if (interpreted_cost != NO_INFORMATION) {
            // ONLY overwrite the layer memory if we have valid gridmap data
            setCost(i, j, interpreted_cost);
          }
        } catch (const std::out_of_range &) {
          // Fallback just in case
        }
      }
      // CRITICAL FIX: If outside the safe zone, DO NOTHING.
      // This leaves your internal layer buffer exactly as it was, preserving
      // memory!
    }
  }

  // Merge the cleanly preserved buffer into the Master Costmap
  if (use_maximum_)
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  else
    updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);

  current_ = true;
  has_updated_data_ = false;

  // Logging Logic
  if (enable_log_) {
    auto end_time = std::chrono::steady_clock::now();
    std::chrono::duration<double, std::milli> duration = end_time - start_time;
    total_execution_time_ += duration.count();
    sample_count_++;

    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_log_time_)
            .count() >= 20) {
      double avg = total_execution_time_ / sample_count_;
      RCLCPP_INFO(
          logger_,
          "GridmapLayer [%s] Avg Update Time: %.3f ms (over %d samples)",
          name_.c_str(), avg, sample_count_);

      total_execution_time_ = 0.0;
      sample_count_ = 0;
      last_log_time_ = now;
    }
  }
}

bool GridmapLayer::getTransform(
    geometry_msgs::msg::TransformStamped &transform) {
  try {
    transform = tf_buffer_->lookupTransform(
        layered_costmap_->getGlobalFrameID(), gridmap_in_.getFrameId(),
        tf2::TimePointZero);
    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN_THROTTLE(logger_, *(clock_), 5000, "TF Error: %s", ex.what());
    return false;
  }
}

} // namespace cprt_costmap_plugins

PLUGINLIB_EXPORT_CLASS(cprt_costmap_plugins::GridmapLayer,
                       nav2_costmap_2d::Layer)