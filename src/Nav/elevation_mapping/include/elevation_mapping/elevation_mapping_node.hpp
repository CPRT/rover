#ifndef ELEVATION_MAP_NODE_HPP
#define ELEVATION_MAP_NODE_HPP

#include <memory>

#include "elevation_mapping/ElevationMapping.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
namespace elevation_mapping {

class ElevationMapNode : public rclcpp::Node {
 public:
  ElevationMapNode(const rclcpp::NodeOptions& opt = rclcpp::NodeOptions());

 private:
  std::unique_ptr<ElevationMapping> elevationMapping_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace elevation_mapping
#endif  // ELEVATION_MAP_NODE_HPP