#include "gstros2src.hpp"
#include "gstros2common.hpp"

#include <gst/gst.h>
#include <gst/video/video.h>

GST_DEBUG_CATEGORY_STATIC(gst_ros2_image_src_debug);
#define GST_CAT_DEFAULT gst_ros2_image_src_debug

enum {
  PROP_0,
  PROP_TOPIC,
  PROP_NODE_NAME,
  PROP_QOS_PROFILE,
  PROP_QUEUE_SIZE,
  PROP_USE_COMPRESSED,
  PROP_OWNS_ROS,
};

#define DEFAULT_TOPIC "/camera/image_raw"
#define DEFAULT_NODE_NAME "gst_ros2_image_src"
#define DEFAULT_QOS_PROFILE "sensor_data"
#define DEFAULT_QUEUE_SIZE 10
#define DEFAULT_USE_COMPRESSED FALSE

G_DEFINE_TYPE(GstRos2ImageSrc, gst_ros2_image_src, GST_TYPE_PUSH_SRC);

// Forward declarations
static void gst_ros2_image_src_set_property(GObject *object, guint prop_id,
                                            const GValue *value,
                                            GParamSpec *pspec);
static void gst_ros2_image_src_get_property(GObject *object, guint prop_id,
                                            GValue *value, GParamSpec *pspec);

static gboolean gst_ros2_image_src_start(GstBaseSrc *src);
static gboolean gst_ros2_image_src_stop(GstBaseSrc *src);
static GstFlowReturn gst_ros2_image_src_create(GstPushSrc *src,
                                               GstBuffer **buf);

static gboolean
gst_ros2_image_src_negotiate_caps(GstRos2ImageSrc *self,
                                  const sensor_msgs::msg::Image &msg);

static GstVideoFormat encoding_to_gst_format(const std::string &encoding);

static void ros_spin_thread_fn(GstRos2ImageSrc *self);

static void gst_ros2_image_src_class_init(GstRos2ImageSrcClass *klass) {
  GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS(klass);
  GstBaseSrcClass *basesrc_class = GST_BASE_SRC_CLASS(klass);
  GstPushSrcClass *pushsrc_class = GST_PUSH_SRC_CLASS(klass);

  gobject_class->set_property = gst_ros2_image_src_set_property;
  gobject_class->get_property = gst_ros2_image_src_get_property;

  // TODO: make topic run-time settable
  g_object_class_install_property(
      gobject_class, PROP_TOPIC,
      g_param_spec_string("topic", "ROS 2 topic",
                          "ROS 2 image topic to subscribe to", DEFAULT_TOPIC,
                          (GParamFlags)(G_PARAM_READWRITE |
                                        G_PARAM_STATIC_STRINGS |
                                        G_PARAM_CONSTRUCT_ONLY)));

  g_object_class_install_property(
      gobject_class, PROP_NODE_NAME,
      g_param_spec_string(
          "node-name", "ROS 2 node name",
          "ROS 2 node name used by this element", DEFAULT_NODE_NAME,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
                        G_PARAM_CONSTRUCT_ONLY)));

  g_object_class_install_property(
      gobject_class, PROP_QOS_PROFILE,
      g_param_spec_string(
          "qos-profile", "QoS profile",
          "QoS profile: 'default' or 'sensor_data'", DEFAULT_QOS_PROFILE,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
                        G_PARAM_CONSTRUCT_ONLY)));

  g_object_class_install_property(
      gobject_class, PROP_QUEUE_SIZE,
      g_param_spec_uint("queue-size", "Queue size",
                        "Number of frames kept in the internal queue", 1, 1000,
                        DEFAULT_QUEUE_SIZE,
                        (GParamFlags)(G_PARAM_READWRITE |
                                      G_PARAM_STATIC_STRINGS |
                                      G_PARAM_CONSTRUCT_ONLY)));
  g_object_class_install_property(
      gobject_class, PROP_USE_COMPRESSED,
      g_param_spec_boolean(
          "use-compressed", "Use compressed",
          "Subscribe to sensor_msgs::msg::CompressedImage "
          "and output compressed buffers (image/jpeg, video/x-h264)",
          DEFAULT_USE_COMPRESSED,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
                        G_PARAM_CONSTRUCT_ONLY)));
  gst_ros2_common::install_owns_ros_property(gobject_class, PROP_OWNS_ROS);

  gst_element_class_set_static_metadata(
      gstelement_class, "ROS 2 Image Source", "Source/Video",
      "Reads ROS 2 image topics and outputs video/x-raw buffers",
      "Connor Needham (CPRT) <connor.needham2015@gmail.com>");

  GstCaps *caps = gst_caps_new_empty();

  gst_caps_append(caps, gst_caps_from_string(GST_VIDEO_CAPS_MAKE(
                            "{ RGB, BGR, GRAY8, RGBA, BGRA }")));

  gst_caps_append(caps, gst_caps_new_simple("image/jpeg", "framerate",
                                            GST_TYPE_FRACTION, 0, 1, NULL));

  gst_caps_append(caps, gst_caps_new_simple("video/x-h264", "framerate",
                                            GST_TYPE_FRACTION, 0, 1, NULL));

  GstPadTemplate *src_templ =
      gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS, caps);
  gst_element_class_add_pad_template(gstelement_class, src_templ);
  gst_caps_unref(caps);

  basesrc_class->start = gst_ros2_image_src_start;
  basesrc_class->stop = gst_ros2_image_src_stop;

  pushsrc_class->create = gst_ros2_image_src_create;

  GST_DEBUG_CATEGORY_INIT(gst_ros2_image_src_debug, "ros2imagesrc", 0,
                          "ROS2 Image Source");
}

static void gst_ros2_image_src_init(GstRos2ImageSrc *self) {
  self->topic = g_strdup(DEFAULT_TOPIC);
  self->node_name = g_strdup(DEFAULT_NODE_NAME);
  self->qos_profile = g_strdup(DEFAULT_QOS_PROFILE);
  self->queue_size = DEFAULT_QUEUE_SIZE;
  self->use_compressed = DEFAULT_USE_COMPRESSED;
  self->owns_ros = DEFAULT_OWNS_ROS;

  self->have_caps = FALSE;
  self->priv = std::make_unique<cpp_ros2_image_src>();
  self->priv->running = false;
  gst_video_info_init(&self->vinfo);

  gst_base_src_set_live(GST_BASE_SRC(self), TRUE);
  gst_base_src_set_format(GST_BASE_SRC(self), GST_FORMAT_TIME);
}

static void gst_ros2_image_src_set_property(GObject *object, guint prop_id,
                                            const GValue *value,
                                            GParamSpec *pspec) {
  GstRos2ImageSrc *self = GST_ROS2_IMAGE_SRC(object);

  switch (prop_id) {
  case PROP_TOPIC:
    g_free(self->topic);
    self->topic = g_value_dup_string(value);
    break;
  case PROP_NODE_NAME:
    g_free(self->node_name);
    self->node_name = g_value_dup_string(value);
    break;
  case PROP_QOS_PROFILE:
    g_free(self->qos_profile);
    self->qos_profile = g_value_dup_string(value);
    break;
  case PROP_QUEUE_SIZE:
    self->queue_size = g_value_get_uint(value);
    break;
  case PROP_USE_COMPRESSED:
    self->use_compressed = g_value_get_boolean(value);
    break;
  case PROP_OWNS_ROS:
    self->owns_ros = g_value_get_boolean(value);
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    break;
  }
}

static void gst_ros2_image_src_get_property(GObject *object, guint prop_id,
                                            GValue *value, GParamSpec *pspec) {
  GstRos2ImageSrc *self = GST_ROS2_IMAGE_SRC(object);

  switch (prop_id) {
  case PROP_TOPIC:
    g_value_set_string(value, self->topic);
    break;
  case PROP_NODE_NAME:
    g_value_set_string(value, self->node_name);
    break;
  case PROP_QOS_PROFILE:
    g_value_set_string(value, self->qos_profile);
    break;
  case PROP_QUEUE_SIZE:
    g_value_set_uint(value, self->queue_size);
    break;
  case PROP_USE_COMPRESSED:
    g_value_set_boolean(value, self->use_compressed);
    break;
  case PROP_OWNS_ROS:
    g_value_set_boolean(value, self->owns_ros);
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    break;
  }
}

static rclcpp::QoS parse_qos_profile(const gchar *qos_profile) {
  if (!qos_profile) {
    return rclcpp::SensorDataQoS();
  }
  std::string q{qos_profile};
  if (q == "default") {
    return rclcpp::QoS(rclcpp::KeepLast(10));
  } else if (q == "sensor_data") {
    return rclcpp::SensorDataQoS();
  }
  // Fallback
  return rclcpp::SensorDataQoS();
}

static GstVideoFormat encoding_to_gst_format(const std::string &encoding) {
  if (encoding == "rgb8" || encoding == "RGB8") {
    return GST_VIDEO_FORMAT_RGB;
  }
  if (encoding == "bgr8" || encoding == "BGR8") {
    return GST_VIDEO_FORMAT_BGR;
  }
  if (encoding == "mono8" || encoding == "MONO8") {
    return GST_VIDEO_FORMAT_GRAY8;
  }
  if (encoding == "rgba8" || encoding == "RGBA8") {
    return GST_VIDEO_FORMAT_RGBA;
  }
  if (encoding == "bgra8" || encoding == "BGRA8") {
    return GST_VIDEO_FORMAT_BGRA;
  }
  return GST_VIDEO_FORMAT_UNKNOWN;
}

static gboolean gst_ros2_image_src_negotiate_caps_compressed(
    GstRos2ImageSrc *self, const sensor_msgs::msg::CompressedImage &msg) {
  const std::string &fmt = msg.format;
  const char *mime = nullptr;

  if (fmt.find("jpeg") != std::string::npos ||
      fmt.find("jpg") != std::string::npos) {
    mime = "image/jpeg";
  } else if (fmt.find("h264") != std::string::npos) {
    mime = "video/x-h264";
  } else {
    GST_ERROR_OBJECT(self, "Unsupported compressed image format: %s",
                     fmt.c_str());
    return FALSE;
  }

  GstCaps *caps =
      gst_caps_new_simple(mime, "framerate", GST_TYPE_FRACTION, 0, 1, NULL);

  if (!caps) {
    GST_ERROR_OBJECT(self, "Failed to create compressed caps");
    return FALSE;
  }

  GST_INFO_OBJECT(self, "Setting src caps (compressed): %" GST_PTR_FORMAT,
                  caps);

  gboolean ret = gst_pad_set_caps(GST_BASE_SRC(self)->srcpad, caps);
  gst_caps_unref(caps);

  self->have_caps = ret;
  return ret;
}

static gboolean
gst_ros2_image_src_negotiate_caps(GstRos2ImageSrc *self,
                                  const sensor_msgs::msg::Image &msg) {
  GstVideoFormat fmt = encoding_to_gst_format(msg.encoding);
  if (fmt == GST_VIDEO_FORMAT_UNKNOWN) {
    GST_ERROR_OBJECT(self, "Unsupported ROS2 image encoding: %s",
                     msg.encoding.c_str());
    return FALSE;
  }

  if (!gst_video_info_set_format(&self->vinfo, fmt, msg.width, msg.height)) {
    GST_ERROR_OBJECT(self, "Failed to set video info");
    return FALSE;
  }

  GstCaps *caps = gst_video_info_to_caps(&self->vinfo);
  if (!caps) {
    GST_ERROR_OBJECT(self, "Failed to create caps from video info");
    return FALSE;
  }

  GST_INFO_OBJECT(self, "Setting src caps: %" GST_PTR_FORMAT, caps);

  gboolean ret = gst_pad_set_caps(GST_BASE_SRC(self)->srcpad, caps);
  gst_caps_unref(caps);

  self->have_caps = ret;
  return ret;
}

static void ros_image_callback(GstRos2ImageSrc *self,
                               const sensor_msgs::msg::Image::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(self->priv->queue_mutex);

  const auto &max_queue_size = self->queue_size;
  auto &is_running = self->priv->running;
  auto &frame_queue = self->priv->frame_queue;
  auto &queue_cv = self->priv->queue_cv;

  if (!is_running.load()) {
    return;
  }

  if (frame_queue.size() >= max_queue_size) {
    frame_queue.pop_front();
  }

  QueuedFrame f;
  f.raw = msg;
  f.compressed.reset();
  frame_queue.push_back(std::move(f));

  lock.unlock();
  queue_cv.notify_one();
}

static void ros_compressed_image_callback(
    GstRos2ImageSrc *self,
    const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(self->priv->queue_mutex);

  const auto &max_queue_size = self->queue_size;
  auto &is_running = self->priv->running;
  auto &frame_queue = self->priv->frame_queue;
  auto &queue_cv = self->priv->queue_cv;

  if (!is_running.load()) {
    return;
  }

  if (frame_queue.size() >= max_queue_size) {
    frame_queue.pop_front();
  }

  QueuedFrame f;
  f.raw.reset();
  f.compressed = msg;
  frame_queue.push_back(std::move(f));

  lock.unlock();
  queue_cv.notify_one();
}

static void ros_spin_thread_fn(GstRos2ImageSrc *self) {
  GST_INFO_OBJECT(self, "ROS spin thread started");
  gst_ros2_common::spin_node(self->priv->node, self->priv->running);
  GST_INFO_OBJECT(self, "ROS spin thread exiting");
}

static gboolean gst_ros2_image_src_start(GstBaseSrc *basesrc) {
  GstRos2ImageSrc *self = GST_ROS2_IMAGE_SRC(basesrc);

  GST_INFO_OBJECT(self, "Starting ROS2 Image Src (use_compressed=%d)",
                  self->use_compressed);

  // Init ROS2 global
  if (!gst_ros2_common::acquire_ros(self->owns_ros)) {
    GST_ERROR_OBJECT(
        self,
        "ROS not initialized. Set owns-ros=true or initialize ROS externally.");
    return FALSE;
  }

  auto &node = self->priv->node;
  auto &frame_queue = self->priv->frame_queue;
  auto &raw_sub = self->priv->raw_sub;
  auto &compressed_sub = self->priv->compressed_sub;
  auto &mutex = self->priv->queue_mutex;
  auto &ros_spin_thread = self->priv->ros_spin_thread;

  rclcpp::NodeOptions opts;
  node = std::make_shared<rclcpp::Node>(
      self->node_name ? self->node_name : DEFAULT_NODE_NAME, opts);

  auto qos = gst_ros2_common::parse_qos_profile(self->qos_profile,
                                                DEFAULT_QOS_PROFILE);

  if (self->use_compressed) {
    auto cb = [self](const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
      ros_compressed_image_callback(self, msg);
    };

    compressed_sub =
        node->create_subscription<sensor_msgs::msg::CompressedImage>(
            self->topic ? self->topic : DEFAULT_TOPIC, qos, cb);
  } else {
    auto cb = [self](const sensor_msgs::msg::Image::SharedPtr msg) {
      ros_image_callback(self, msg);
    };

    raw_sub = node->create_subscription<sensor_msgs::msg::Image>(
        self->topic ? self->topic : DEFAULT_TOPIC, qos, cb);
  }

  {
    std::lock_guard<std::mutex> lock(mutex);
    frame_queue.clear();
  }

  self->have_caps = FALSE;
  self->priv->running.store(true);

  ros_spin_thread = std::thread(ros_spin_thread_fn, self);

  return TRUE;
}

static gboolean gst_ros2_image_src_stop(GstBaseSrc *basesrc) {
  GstRos2ImageSrc *self = GST_ROS2_IMAGE_SRC(basesrc);

  GST_INFO_OBJECT(self, "Stopping ROS2 Image Src");

  self->priv->running.store(false);
  self->priv->queue_cv.notify_all();

  if (self->priv->ros_spin_thread.joinable()) {
    self->priv->ros_spin_thread.join();
  }

  self->priv->raw_sub.reset();
  self->priv->compressed_sub.reset();
  self->priv->node.reset();

  gst_ros2_common::release_ros(self->owns_ros);

  {
    std::lock_guard<std::mutex> lock(self->priv->queue_mutex);
    self->priv->frame_queue.clear();
  }

  return TRUE;
}

static GstFlowReturn gst_ros2_image_src_create(GstPushSrc *pushsrc,
                                               GstBuffer **out_buf) {
  GstRos2ImageSrc *self = GST_ROS2_IMAGE_SRC(pushsrc);

  QueuedFrame frame;

  {
    std::unique_lock<std::mutex> lock(self->priv->queue_mutex);

    self->priv->queue_cv.wait(lock, [self]() {
      return !self->priv->frame_queue.empty() || !self->priv->running.load();
    });

    if (!self->priv->running.load() && self->priv->frame_queue.empty()) {
      GST_INFO_OBJECT(self, "Stopping create because element is not running");
      return GST_FLOW_EOS;
    }

    frame = self->priv->frame_queue.front();
    self->priv->frame_queue.pop_front();
  }

  if (self->use_compressed) {
    auto msg = frame.compressed;
    if (!msg) {
      GST_WARNING_OBJECT(self, "Got null compressed image message");
      return GST_FLOW_ERROR;
    }

    // Negotiate caps if needed
    if (!self->have_caps) {
      if (!gst_ros2_image_src_negotiate_caps_compressed(self, *msg)) {
        GST_ERROR_OBJECT(self, "Failed to negotiate compressed caps");
        return GST_FLOW_ERROR;
      }
    }

    gsize buffer_size = msg->data.size();
    if (buffer_size == 0) {
      GST_WARNING_OBJECT(self, "Compressed image has zero data size");
      return GST_FLOW_ERROR;
    }

    GstBuffer *buffer = gst_buffer_new_and_alloc(buffer_size);
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
      GST_ERROR_OBJECT(self, "Failed to map buffer for writing (compressed)");
      gst_buffer_unref(buffer);
      return GST_FLOW_ERROR;
    }

    std::memcpy(map.data, msg->data.data(), buffer_size);
    gst_buffer_unmap(buffer, &map);

    GstClockTime pts = gst_element_get_current_running_time(GST_ELEMENT(self));
    GST_BUFFER_PTS(buffer) = pts;
    GST_BUFFER_DTS(buffer) = pts;
    GST_BUFFER_DURATION(buffer) = GST_CLOCK_TIME_NONE;

    *out_buf = buffer;
    return GST_FLOW_OK;
  }

  // Raw image processing
  auto msg = frame.raw;
  if (!msg) {
    GST_WARNING_OBJECT(self, "Got null ROS2 image message");
    return GST_FLOW_ERROR;
  }

  if (!self->have_caps) {
    if (!gst_ros2_image_src_negotiate_caps(self, *msg)) {
      GST_ERROR_OBJECT(self, "Failed to negotiate caps");
      return GST_FLOW_ERROR;
    }
  }

  gsize expected_size = GST_VIDEO_INFO_SIZE(&self->vinfo);
  if (expected_size != msg->data.size()) {
    GST_WARNING_OBJECT(self,
                       "Data size mismatch (expected %zu, got %zu). "
                       "Continuing with min size.",
                       (size_t)expected_size, (size_t)msg->data.size());
  }

  gsize buffer_size = std::min<gsize>(expected_size, msg->data.size());

  GstBuffer *buffer = gst_buffer_new_and_alloc(buffer_size);
  GstMapInfo map;
  if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE)) {
    GST_ERROR_OBJECT(self, "Failed to map buffer for writing");
    gst_buffer_unref(buffer);
    return GST_FLOW_ERROR;
  }

  std::memcpy(map.data, msg->data.data(), buffer_size);
  gst_buffer_unmap(buffer, &map);

  GstClockTime pts = gst_element_get_current_running_time(GST_ELEMENT(self));
  GST_BUFFER_PTS(buffer) = pts;
  GST_BUFFER_DTS(buffer) = pts;
  GST_BUFFER_DURATION(buffer) = GST_CLOCK_TIME_NONE;

  *out_buf = buffer;
  return GST_FLOW_OK;
}
