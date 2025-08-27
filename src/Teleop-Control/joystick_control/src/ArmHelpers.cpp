#include "ArmHelpers.hpp"

#include "std_srvs/srv/trigger.hpp"

bool ArmHelpers::start_moveit_servo(rclcpp::Node* node, int attempts) {
  auto start_moveit_client =
      node->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  int i = 0;
  while (!start_moveit_client->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_WARN(node->get_logger(), "Service not available, Waiting...");
    if (++i >= attempts) {
      RCLCPP_ERROR(node->get_logger(),
                   "Service not available after %d attempts. Giving up", i);
      return false;
    }
  }
  auto start_request = std::make_shared<std_srvs::srv::Trigger::Request>();
  start_moveit_client->async_send_request(start_request);
  return true;
}
