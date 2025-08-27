#include "ros_phoenix/phoenix_components.hpp"

// Register components
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ros_phoenix::TalonFX)
RCLCPP_COMPONENTS_REGISTER_NODE(ros_phoenix::TalonSRX)
RCLCPP_COMPONENTS_REGISTER_NODE(ros_phoenix::VictorSPX)