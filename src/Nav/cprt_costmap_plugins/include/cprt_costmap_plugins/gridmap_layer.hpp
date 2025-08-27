
#ifndef NAV2_GRIDMAP_COSTMAP_GRIDMAP_LAYER_HPP_
#define NAV2_GRIDMAP_COSTMAP_GRIDMAP_LAYER_HPP_

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <mutex>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/footprint.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

namespace cprt_costmap_plugins {

/**
 * @class GridmapLayer
 * @brief Takes in a gridmap map generated to add costs to costmap
 */
class GridmapLayer : public nav2_costmap_2d::CostmapLayer {
 public:
  /**
   * @brief Gridmap Layer constructor
   */
  GridmapLayer();
  /**
   * @brief Gridmap Layer destructor
   */
  virtual ~GridmapLayer();

  /**
   * @brief Initialization process of layer on startup
   */
  virtual void onInitialize();

  /**
   * @brief Activate this layer
   */
  virtual void activate();
  /**
   * @brief Deactivate this layer
   */
  virtual void deactivate();

  /**
   * @brief Reset this costmap
   */
  virtual void reset();

  /**
   * @brief Matches the layer size to the master size
   */
  virtual void matchSize();

  /**
   * @brief If clearing operations should be processed on this layer or not
   */
  virtual bool isClearable() { return false; }

  /**
   * @brief Update the bounds of the master costmap by this layer's update
   * dimensions
   * @param robot_x X pose of robot
   * @param robot_y Y pose of robot
   * @param robot_yaw Robot orientation
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
                            double* min_x, double* min_y, double* max_x,
                            double* max_y);

  /**
   * @brief Update the costs in the master costmap in the window
   * @param master_grid The master costmap grid to update
   * @param min_x X min map coord of the window to update
   * @param min_y Y min map coord of the window to update
   * @param max_x X max map coord of the window to update
   * @param max_y Y max map coord of the window to update
   */
  virtual void updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i,
                           int min_j, int max_i, int max_j);

 protected:
  /**
   * @brief Get parameters of layer
   */
  void getParameters();

  /**
   * @brief Process a new map coming from a topic
   */
  void processMap(const grid_map::GridMap& new_map);

  /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
  void incomingMap(const grid_map_msgs::msg::GridMap::SharedPtr new_map);

  /**
   * @brief Interpret the value in the static map given on the topic to
   * convert into costs for the costmap to utilize
   */
  unsigned char interpretValue(double value);

  std::vector<geometry_msgs::msg::Point> transformed_footprint_;
  /**
   * @brief Clear costmap layer info below the robot's footprint
   */
  void updateFootprint(double robot_x, double robot_y, double robot_yaw,
                       double* min_x, double* min_y, double* max_x,
                       double* max_y);

  bool getTransform(geometry_msgs::msg::TransformStamped& transform);

  std::string global_frame_;  ///< @brief The global frame for the costmap
  std::string map_frame_;     /// @brief frame that map is located in

  bool has_updated_data_{false};

  double x_{0};
  double y_{0};
  double width_{0};
  double height_{0};

  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr map_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Parameters
  std::string map_topic_;
  std::string layer_name_;
  bool track_unknown_space_;
  bool use_maximum_;
  unsigned char lethal_threshold_;
  unsigned char unknown_cost_value_;
  bool trinary_costmap_;
  bool map_received_{false};
  bool map_received_in_update_bounds_{false};
  tf2::Duration transform_tolerance_;
  grid_map::GridMap gridmap_in_;
};

}  // namespace cprt_costmap_plugins

#endif  // NAV2_GRIDMAP_COSTMAP_GRIDMAP_LAYER_HPP_