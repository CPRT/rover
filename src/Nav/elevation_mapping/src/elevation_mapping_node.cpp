/*
 * elevation_map_node.cpp
 *
 *  Created on: Oct 3, 2013
 *      Author: Péter Fankhauser
 *   Institute: ETH Zurich, ANYbotics
 */

#include "elevation_mapping/elevation_mapping_node.hpp"

#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;

namespace elevation_mapping {

ElevationMapNode::ElevationMapNode(const rclcpp::NodeOptions& opt)
    : Node("elevation_mapping", opt) {
  declare_parameter("num_callback_threads", 5);
  declare_parameter("postprocessor_num_threads", 1);
  timer_ = this->create_wall_timer(2s, [this]() {
    this->timer_->cancel();
    elevationMapping_ = std::make_unique<ElevationMapping>(shared_from_this());
  });
}
}  // namespace elevation_mapping

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<elevation_mapping::ElevationMapNode>();
  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions(),
      node->get_parameter("num_callback_threads").as_int());
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(elevation_mapping::ElevationMapNode)