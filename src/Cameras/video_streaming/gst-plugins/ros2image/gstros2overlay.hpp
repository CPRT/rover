#ifndef __GST_ROS2_OVERLAY_HPP__
#define __GST_ROS2_OVERLAY_HPP__

#include <gst/gst.h>
#include <gst/video/gstvideofilter.h>
#include <gst/video/video.h>

#include <geometry_msgs/msg/vector3.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

G_BEGIN_DECLS

#define GST_TYPE_ROS2_OVERLAY (gst_ros2_overlay_get_type())
#define GST_ROS2_OVERLAY(obj)                                                  \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_ROS2_OVERLAY, GstRos2Overlay))
#define GST_ROS2_OVERLAY_CLASS(klass)                                          \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_ROS2_OVERLAY, GstRos2OverlayClass))
#define GST_IS_ROS2_OVERLAY(obj)                                               \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_ROS2_OVERLAY))
#define GST_IS_ROS2_OVERLAY_CLASS(klass)                                       \
  (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_ROS2_OVERLAY))

typedef struct _GstRos2Overlay GstRos2Overlay;
typedef struct _GstRos2OverlayClass GstRos2OverlayClass;

struct cpp_ros2_overlay {
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr text_sub;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr dot_sub;
  std::thread ros_spin_thread;
  std::atomic<bool> running{false};

  std::mutex data_mutex;
  std::string current_text;
  bool have_text{false};
  double dot_x{0.0};
  double dot_y{0.0};
  bool have_dot{false};
};

struct _GstRos2Overlay {
  GstVideoFilter parent;

  gchar *node_name;
  gchar *text_topic;
  gchar *dot_topic;
  gchar *qos_profile;
  gboolean owns_ros;

  GstVideoInfo info;
  std::unique_ptr<cpp_ros2_overlay> priv;
};

struct _GstRos2OverlayClass {
  GstVideoFilterClass parent_class;
};

GType gst_ros2_overlay_get_type(void);

G_END_DECLS

#endif