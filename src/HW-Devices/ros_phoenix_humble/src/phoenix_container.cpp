#include <memory>
#include <string>

#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"
#include "ros_phoenix/phoenix_manager.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto phoenix_manager = ros_phoenix::PhoenixManager::getInstance(exec);

  exec->add_node(phoenix_manager);
  exec->spin();

  return 0;
}