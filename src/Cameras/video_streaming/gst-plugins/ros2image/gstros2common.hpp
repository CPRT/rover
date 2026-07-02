#ifndef GST_ROS2_COMMON_HPP
#define GST_ROS2_COMMON_HPP

#include <gst/gst.h>

#include <rclcpp/rclcpp.hpp>

#include <atomic>
#include <memory>

namespace gst_ros2_common {

#define DEFAULT_OWNS_ROS TRUE

void install_owns_ros_property(GObjectClass *gobject_class, guint prop_id);

rclcpp::QoS parse_qos_profile(const gchar *qos_profile,
                              const gchar *default_profile);

bool acquire_ros(bool owns_ros);

void release_ros(bool owns_ros);

void spin_node(const rclcpp::Node::SharedPtr &node,
               const std::atomic<bool> &running);

} // namespace gst_ros2_common

#endif // GST_ROS2_COMMON_HPP