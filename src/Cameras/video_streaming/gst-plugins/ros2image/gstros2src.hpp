#ifndef __GST_ROS2_IMAGE_SRC_HPP__
#define __GST_ROS2_IMAGE_SRC_HPP__

#include <gst/base/gstpushsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>

G_BEGIN_DECLS

#define GST_TYPE_ROS2_IMAGE_SRC (gst_ros2_image_src_get_type())
#define GST_ROS2_IMAGE_SRC(obj)                                                \
  (G_TYPE_CHECK_INSTANCE_CAST((obj), GST_TYPE_ROS2_IMAGE_SRC, GstRos2ImageSrc))
#define GST_ROS2_IMAGE_SRC_CLASS(klass)                                        \
  (G_TYPE_CHECK_CLASS_CAST((klass), GST_TYPE_ROS2_IMAGE_SRC,                   \
                           GstRos2ImageSrcClass))
#define GST_IS_ROS2_IMAGE_SRC(obj)                                             \
  (G_TYPE_CHECK_INSTANCE_TYPE((obj), GST_TYPE_ROS2_IMAGE_SRC))
#define GST_IS_ROS2_IMAGE_SRC_CLASS(klass)                                     \
  (G_TYPE_CHECK_CLASS_TYPE((klass), GST_TYPE_ROS2_IMAGE_SRC))

typedef struct _GstRos2ImageSrc GstRos2ImageSrc;
typedef struct _GstRos2ImageSrcClass GstRos2ImageSrcClass;

// Generic queued frame structure
// Holds either raw or compressed image message
struct QueuedFrame {
  sensor_msgs::msg::Image::SharedPtr raw;
  sensor_msgs::msg::CompressedImage::SharedPtr compressed;
};

// _GstRos2ImageSrc is created by gstreamer (c library) and malloc does not
// initialize cpp classes correctly.
struct cpp_ros2_image_src {
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
      compressed_sub;
  std::thread ros_spin_thread;
  std::atomic<bool> running;

  std::mutex queue_mutex;
  std::condition_variable queue_cv;
  std::deque<QueuedFrame> frame_queue;
};

struct _GstRos2ImageSrc {
  GstPushSrc parent;

  gchar *topic;
  gchar *node_name;
  gchar *qos_profile;
  guint queue_size;
  gboolean use_compressed;

  GstVideoInfo vinfo;
  gboolean have_caps;
  std::unique_ptr<cpp_ros2_image_src> priv;
};

struct _GstRos2ImageSrcClass {
  GstPushSrcClass parent_class;
};

GType gst_ros2_image_src_get_type(void);

G_END_DECLS

#endif /* __GST_ROS2_IMAGE_SRC_HPP__ */
