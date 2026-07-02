#include "gstros2common.hpp"

#include <chrono>
#include <mutex>
#include <string>
#include <thread>

namespace gst_ros2_common {

static std::atomic<int> g_ros2_owner_count{0};
static std::mutex g_ros2_init_mutex;

void install_owns_ros_property(GObjectClass *gobject_class, guint prop_id) {
  g_object_class_install_property(
      gobject_class, prop_id,
      g_param_spec_boolean(
          "owns-ros", "Owns ROS",
          "If true, element will call rclcpp::init/shutdown. "
          "Set false when running inside an existing ROS2 process.",
          DEFAULT_OWNS_ROS,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
                        G_PARAM_CONSTRUCT_ONLY)));
}

rclcpp::QoS parse_qos_profile(const gchar *qos_profile,
                              const gchar *default_profile) {
  const std::string profile =
      qos_profile ? qos_profile
                  : (default_profile ? default_profile : "default");

  if (profile == "sensor_data") {
    return rclcpp::SensorDataQoS();
  }

  return rclcpp::QoS(rclcpp::KeepLast(10));
}

bool acquire_ros(bool owns_ros) {
  if (!owns_ros) {
    return rclcpp::ok();
  }

  std::lock_guard<std::mutex> lock(g_ros2_init_mutex);

  if (g_ros2_owner_count.fetch_add(1) == 0 && !rclcpp::ok()) {
    int argc = 0;
    char **argv = nullptr;
    rclcpp::init(argc, argv);
  }

  return rclcpp::ok();
}

void release_ros(bool owns_ros) {
  if (!owns_ros) {
    return;
  }

  std::lock_guard<std::mutex> lock(g_ros2_init_mutex);

  if (g_ros2_owner_count.fetch_sub(1) == 1 && rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void spin_node(const rclcpp::Node::SharedPtr &node,
               const std::atomic<bool> &running) {
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  while (running.load()) {
    exec.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  exec.remove_node(node);
}

} // namespace gst_ros2_common