
#include "cprt_costmap_plugins/gridmap_layer.hpp"

#include <tf2/convert.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <pluginlib/class_list_macros.hpp>
#include <string>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using nav2_costmap_2d::FREE_SPACE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace cprt_costmap_plugins {

GridmapLayer::GridmapLayer() {}

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
  Costmap2D *master = layered_costmap_->getCostmap();
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
            master->getResolution(), master->getOriginX(),
            master->getOriginY());
  RCLCPP_INFO(logger_, "sx:%f, sy: %f, ox: %f, oy %f", getSizeInMetersX(),
              getSizeInMetersY(), getOriginX(), getOriginY());
}

void GridmapLayer::getParameters() {
  int temp_lethal_threshold = 0;
  double temp_tf_tol = 0.0;

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("map_subscribe_transient_local",
                   rclcpp::ParameterValue(true));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));
  declareParameter("map_topic", rclcpp::ParameterValue("/map"));
  declareParameter("layer_name", rclcpp::ParameterValue("traversability_map"));
  declareParameter("footprint_clearing_enabled", rclcpp::ParameterValue(false));

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"Failed to lock node"};
  }

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "map_topic", map_topic_);
  node->get_parameter(name_ + "." + "layer_name", layer_name_);
  node->get_parameter("track_unknown_space", track_unknown_space_);
  node->get_parameter("use_maximum", use_maximum_);
  node->get_parameter("lethal_cost_threshold", temp_lethal_threshold);
  node->get_parameter("unknown_cost_value", unknown_cost_value_);
  node->get_parameter("trinary_costmap", trinary_costmap_);
  node->get_parameter("transform_tolerance", temp_tf_tol);

  // Enforce bounds
  lethal_threshold_ = std::max(std::min(temp_lethal_threshold, 255), 0);
  map_received_ = false;

  transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);
}

unsigned char GridmapLayer::interpretValue(double value) {
  // Gridmap unknown is NaN
  if (!std::isfinite(value)) {
    return track_unknown_space_ ? NO_INFORMATION : FREE_SPACE;
  }
  value = value * 255;
  if (value >= lethal_threshold_) {
    return LETHAL_OBSTACLE;
  }
  if (trinary_costmap_) {
    return FREE_SPACE;
  }

  const double scale = value / lethal_threshold_;
  return scale * LETHAL_OBSTACLE;
}

void GridmapLayer::incomingMap(
    const grid_map_msgs::msg::GridMap::SharedPtr new_map) {
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
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

  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());

  if (layered_costmap_->isRolling()) {
    updateOrigin(robot_x - getSizeInMetersX() / 2,
                 robot_y - getSizeInMetersY() / 2);
  }

  if (!layered_costmap_->isRolling()) {
    if (!(has_updated_data_ || has_extra_bounds_)) {
      return;
    }
  }

  useExtraBounds(min_x, min_y, max_x, max_y);
  geometry_msgs::msg::TransformStamped transform;
  if (!getTransform(transform)) {
    return;
  }
  geometry_msgs::msg::PointStamped grid_map_point, costmap_point;
  width_ = gridmap_in_.getLength().x();
  height_ = gridmap_in_.getLength().y();
  grid_map_point.point.x = gridmap_in_.getPosition().x();
  grid_map_point.point.y = gridmap_in_.getPosition().y();
  grid_map_point.point.z = 0.0;
  tf2::doTransform(grid_map_point, costmap_point, transform);
  x_ = costmap_point.point.x;
  y_ = costmap_point.point.y;

  const double rad_x = width_ * 0.5;
  const double rad_y = height_ * 0.5;
  *min_x = std::min(x_ - rad_x, *min_x);
  *min_y = std::min(y_ - rad_y, *min_y);
  *max_x = std::max(x_ + rad_x, *max_x);
  *max_y = std::max(y_ + rad_y, *max_y);
}

bool GridmapLayer::getTransform(
    geometry_msgs::msg::TransformStamped &transform) {
  try {
    transform = tf_buffer_->lookupTransform(
        layered_costmap_->getGlobalFrameID(), gridmap_in_.getFrameId(),
        tf2::TimePointZero);
    return true;
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(logger_, "Gridmap layer: %s", ex.what());
    return false;
  }
}

void GridmapLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                               int min_i, int min_j, int max_i, int max_j) {
  std::lock_guard<Costmap2D::mutex_t> guard(*getMutex());
  if (!enabled_) {
    return;
  }

  // Don't write to master costmap until we've received at least one map
  if (!map_received_in_update_bounds_) {
    static int count = 0;
    // throttle warning down to only 1/10 message rate
    if (++count == 10) {
      RCLCPP_WARN(logger_,
                  "Can't update gridmap costmap layer, no map received");
      count = 0;
    }
    return;
  }

  // FIX: For rolling costmaps, we MUST re-project every cycle to prevent
  // flickering For static costmaps, only update when new data arrives
  if (!has_updated_data_ && !layered_costmap_->isRolling()) {
    // No new data and not rolling - skip the update loop but still merge
    if (use_maximum_) {
      updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    } else {
      updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
    }
    current_ = true;
    return;
  }

  geometry_msgs::msg::TransformStamped transform_stamped;
  if (!getTransform(transform_stamped)) {
    return;
  }

  // Check if the required layer exists in the grid map to avoid exceptions
  if (!gridmap_in_.exists(layer_name_)) {
    RCLCPP_ERROR(logger_,
                 "GridMap layer '%s' does not exist in received grid map. "
                 "Available layers: [%s]. Check your configuration.",
                 layer_name_.c_str(),
                 [&]() {
                   std::string layers;
                   for (const auto &layer : gridmap_in_.getLayers()) {
                     if (!layers.empty())
                       layers += ", ";
                     layers += layer;
                   }
                   return layers;
                 }()
                     .c_str());
    return;
  }

  // OPTIMIZATION: Convert transform once before the loop to avoid repeated
  // conversions
  tf2::Transform tf2_transform;
  tf2::fromMsg(transform_stamped.transform, tf2_transform);

  // Iterate through the grid map and copy values to the internal costmap
  for (grid_map::GridMapIterator it(gridmap_in_); !it.isPastEnd(); ++it) {
    const grid_map::Index index(*it);
    const float value = gridmap_in_.at(layer_name_, index);
    const auto cost = interpretValue(value);

    // PRESERVE DATA: Skip unknown values to prevent overwriting valid obstacles
    if (cost == NO_INFORMATION) {
      continue;
    }

    // Convert grid_map index to world coordinates
    grid_map::Position position;
    gridmap_in_.getPosition(index, position);

    // OPTIMIZATION: Use direct tf2::Transform multiplication instead of
    // doTransform
    tf2::Vector3 grid_map_point(position.x(), position.y(), 0.0);
    tf2::Vector3 costmap_point = tf2_transform * grid_map_point;

    // Convert world coordinates to costmap cell coordinates
    unsigned int mx, my;
    const bool isValid =
        worldToMap(costmap_point.x(), costmap_point.y(), mx, my);
    if (isValid) {
      // Write to the layer's own internal buffer (not the master)
      setCost(mx, my, cost);
    }
  }

  // Mark as processed after successful copy
  has_updated_data_ = false;

  // Always merge the internal buffer into the master costmap
  if (use_maximum_) {
    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  } else {
    updateWithTrueOverwrite(master_grid, min_i, min_j, max_i, max_j);
  }
  current_ = true;
}

} // namespace cprt_costmap_plugins

PLUGINLIB_EXPORT_CLASS(cprt_costmap_plugins::GridmapLayer,
                       nav2_costmap_2d::Layer)