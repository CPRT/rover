#include "interface_manager.h"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InterfaceManagerNode>("interface_manager_node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}