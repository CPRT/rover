#include "gstros2overlay.hpp"
#include "gstros2common.hpp"

#include <gst/video/video-frame.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstring>

GST_DEBUG_CATEGORY_STATIC(gst_ros2_overlay_debug);
#define GST_CAT_DEFAULT gst_ros2_overlay_debug

enum {
  PROP_0,
  PROP_NODE_NAME,
  PROP_TEXT_TOPIC,
  PROP_DOT_TOPIC,
  PROP_QOS_PROFILE,
  PROP_OWNS_ROS,
};

#define DEFAULT_NODE_NAME "gst_ros2_overlay"
#define DEFAULT_TEXT_TOPIC "/overlay/text"
#define DEFAULT_DOT_TOPIC "/overlay/dot"
#define DEFAULT_QOS_PROFILE "default"

G_DEFINE_TYPE(GstRos2Overlay, gst_ros2_overlay, GST_TYPE_VIDEO_FILTER);

static void gst_ros2_overlay_set_property(GObject *object, guint prop_id,
                                          const GValue *value,
                                          GParamSpec *pspec);
static void gst_ros2_overlay_get_property(GObject *object, guint prop_id,
                                          GValue *value, GParamSpec *pspec);
static void gst_ros2_overlay_finalize(GObject *object);

static gboolean gst_ros2_overlay_start(GstBaseTransform *trans);
static gboolean gst_ros2_overlay_stop(GstBaseTransform *trans);
static gboolean gst_ros2_overlay_set_info(GstVideoFilter *filter,
                                          GstCaps *incaps, GstVideoInfo *ininfo,
                                          GstCaps *outcaps,
                                          GstVideoInfo *outinfo);
static GstFlowReturn gst_ros2_overlay_transform_frame_ip(GstVideoFilter *filter,
                                                         GstVideoFrame *frame);

static void ros_spin_thread_fn(GstRos2Overlay *self);

static void text_callback(GstRos2Overlay *self,
                          const std_msgs::msg::String::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(self->priv->data_mutex);
  self->priv->current_text = msg->data;
  self->priv->have_text = true;
}

static void dot_callback(GstRos2Overlay *self,
                         const geometry_msgs::msg::Vector3::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(self->priv->data_mutex);
  if (msg->x < 0 || msg->y < 0) {
    self->priv->have_dot = false;
    return;
  }
  self->priv->dot_x = msg->x;
  self->priv->dot_y = msg->y;
  self->priv->have_dot = true;
}

static void ros_spin_thread_fn(GstRos2Overlay *self) {
    gst_ros2_common::spin_node(self->priv->node, self->priv->running);
}

static void put_pixel_bgr(guint8 *data, int stride, int width, int height,
                          int x, int y, guint8 b, guint8 g, guint8 r) {
  if (x < 0 || y < 0 || x >= width || y >= height) {
    return;
  }

  guint8 *p = data + y * stride + x * 3;
  p[0] = b;
  p[1] = g;
  p[2] = r;
}

static void put_pixel_rgb(guint8 *data, int stride, int width, int height,
                          int x, int y, guint8 r, guint8 g, guint8 b) {
  if (x < 0 || y < 0 || x >= width || y >= height) {
    return;
  }

  guint8 *p = data + y * stride + x * 3;
  p[0] = r;
  p[1] = g;
  p[2] = b;
}

static void put_pixel_rgba(guint8 *data, int stride, int width, int height,
                           int x, int y, guint8 r, guint8 g, guint8 b) {
  if (x < 0 || y < 0 || x >= width || y >= height) {
    return;
  }

  guint8 *p = data + y * stride + x * 4;
  p[0] = r;
  p[1] = g;
  p[2] = b;
  // preserve existing alpha
}

static void put_pixel_gray(guint8 *data, int stride, int width, int height,
                           int x, int y, guint8 v) {
  if (x < 0 || y < 0 || x >= width || y >= height) {
    return;
  }

  guint8 *p = data + y * stride + x;
  p[0] = v;
}

static void draw_dot_bgr(guint8 *data, int stride, int width, int height,
                         int cx, int cy, int radius) {
  for (int y = cy - radius; y <= cy + radius; ++y) {
    for (int x = cx - radius; x <= cx + radius; ++x) {
      int dx = x - cx;
      int dy = y - cy;
      if (dx * dx + dy * dy <= radius * radius) {
        put_pixel_bgr(data, stride, width, height, x, y, 0, 0, 255);
      }
    }
  }
}

static void draw_dot_rgb(guint8 *data, int stride, int width, int height,
                         int cx, int cy, int radius) {
  for (int y = cy - radius; y <= cy + radius; ++y) {
    for (int x = cx - radius; x <= cx + radius; ++x) {
      int dx = x - cx;
      int dy = y - cy;
      if (dx * dx + dy * dy <= radius * radius) {
        put_pixel_rgb(data, stride, width, height, x, y, 255, 0, 0);
      }
    }
  }
}
static void draw_dot_rgba(guint8 *data, int stride, int width, int height,
                          int cx, int cy, int radius) {
  for (int y = cy - radius; y <= cy + radius; ++y) {
    for (int x = cx - radius; x <= cx + radius; ++x) {
      int dx = x - cx;
      int dy = y - cy;
      if (dx * dx + dy * dy <= radius * radius) {
        put_pixel_rgba(data, stride, width, height, x, y, 255, 0, 0);
      }
    }
  }
}

static void draw_dot_gray(guint8 *data, int stride, int width, int height,
                          int cx, int cy, int radius) {
  for (int y = cy - radius; y <= cy + radius; ++y) {
    for (int x = cx - radius; x <= cx + radius; ++x) {
      int dx = x - cx;
      int dy = y - cy;
      if (dx * dx + dy * dy <= radius * radius) {
        put_pixel_gray(data, stride, width, height, x, y, 255);
      }
    }
  }
}

/*
  Tiny 5x7 bitmap font for a practical subset of ASCII.
  Unsupported chars render as space.
*/
static const uint8_t FONT_5X7[][5] = {
    {0, 0, 0, 0, 0},                // space 32
    {0x00, 0x00, 0x5F, 0x00, 0x00}, // !
    {0x00, 0x07, 0x00, 0x07, 0x00}, // "
    {0x14, 0x7F, 0x14, 0x7F, 0x14}, // #
    {0x24, 0x2A, 0x7F, 0x2A, 0x12}, // $
    {0x23, 0x13, 0x08, 0x64, 0x62}, // %
    {0x36, 0x49, 0x55, 0x22, 0x50}, // &
    {0x00, 0x05, 0x03, 0x00, 0x00}, // '
    {0x00, 0x1C, 0x22, 0x41, 0x00}, // (
    {0x00, 0x41, 0x22, 0x1C, 0x00}, // )
    {0x14, 0x08, 0x3E, 0x08, 0x14}, // *
    {0x08, 0x08, 0x3E, 0x08, 0x08}, // +
    {0x00, 0x50, 0x30, 0x00, 0x00}, // ,
    {0x08, 0x08, 0x08, 0x08, 0x08}, // -
    {0x00, 0x60, 0x60, 0x00, 0x00}, // .
    {0x20, 0x10, 0x08, 0x04, 0x02}, // /
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // 0
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // 1
    {0x42, 0x61, 0x51, 0x49, 0x46}, // 2
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // 3
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // 4
    {0x27, 0x45, 0x45, 0x45, 0x39}, // 5
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // 6
    {0x01, 0x71, 0x09, 0x05, 0x03}, // 7
    {0x36, 0x49, 0x49, 0x49, 0x36}, // 8
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // 9
    {0x00, 0x36, 0x36, 0x00, 0x00}, // :
    {0x00, 0x56, 0x36, 0x00, 0x00}, // ;
    {0x08, 0x14, 0x22, 0x41, 0x00}, // <
    {0x14, 0x14, 0x14, 0x14, 0x14}, // =
    {0x00, 0x41, 0x22, 0x14, 0x08}, // >
    {0x02, 0x01, 0x51, 0x09, 0x06}, // ?
    {0x32, 0x49, 0x79, 0x41, 0x3E}, // @
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // A
    {0x7F, 0x49, 0x49, 0x49, 0x36}, // B
    {0x3E, 0x41, 0x41, 0x41, 0x22}, // C
    {0x7F, 0x41, 0x41, 0x22, 0x1C}, // D
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // E
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // F
    {0x3E, 0x41, 0x49, 0x49, 0x7A}, // G
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // H
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // I
    {0x20, 0x40, 0x41, 0x3F, 0x01}, // J
    {0x7F, 0x08, 0x14, 0x22, 0x41}, // K
    {0x7F, 0x40, 0x40, 0x40, 0x40}, // L
    {0x7F, 0x02, 0x0C, 0x02, 0x7F}, // M
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // N
    {0x3E, 0x41, 0x41, 0x41, 0x3E}, // O
    {0x7F, 0x09, 0x09, 0x09, 0x06}, // P
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // Q
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // R
    {0x46, 0x49, 0x49, 0x49, 0x31}, // S
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // T
    {0x3F, 0x40, 0x40, 0x40, 0x3F}, // U
    {0x1F, 0x20, 0x40, 0x20, 0x1F}, // V
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // W
    {0x63, 0x14, 0x08, 0x14, 0x63}, // X
    {0x03, 0x04, 0x78, 0x04, 0x03}, // Y
    {0x61, 0x51, 0x49, 0x45, 0x43}, // Z
};

static const uint8_t *font5x7_for_char(char c) {
  if (c < 32 || c > 90) {
    return FONT_5X7[0];
  }
  return FONT_5X7[c - 32];
}

static void draw_char_bgr(guint8 *data, int stride, int width, int height,
                          int x, int y, char c) {
  const uint8_t *glyph = font5x7_for_char(c);
  for (int col = 0; col < 5; ++col) {
    for (int row = 0; row < 7; ++row) {
      if (glyph[col] & (1 << row)) {
        put_pixel_bgr(data, stride, width, height, x + col, y + row, 0, 255, 0);
      }
    }
  }
}

static void draw_char_rgb(guint8 *data, int stride, int width, int height,
                          int x, int y, char c) {
  const uint8_t *glyph = font5x7_for_char(c);
  for (int col = 0; col < 5; ++col) {
    for (int row = 0; row < 7; ++row) {
      if (glyph[col] & (1 << row)) {
        put_pixel_rgb(data, stride, width, height, x + col, y + row, 0, 255, 0);
      }
    }
  }
}

static void draw_char_rgba(guint8 *data, int stride, int width, int height,
                           int x, int y, char c) {
  const uint8_t *glyph = font5x7_for_char(c);
  for (int col = 0; col < 5; ++col) {
    for (int row = 0; row < 7; ++row) {
      if (glyph[col] & (1 << row)) {
        put_pixel_rgba(data, stride, width, height, x + col, y + row, 0, 255,
                       0);
      }
    }
  }
}

static void draw_char_gray(guint8 *data, int stride, int width, int height,
                           int x, int y, char c) {
  const uint8_t *glyph = font5x7_for_char(c);
  for (int col = 0; col < 5; ++col) {
    for (int row = 0; row < 7; ++row) {
      if (glyph[col] & (1 << row)) {
        put_pixel_gray(data, stride, width, height, x + col, y + row, 255);
      }
    }
  }
}

static void draw_text_bgr(guint8 *data, int stride, int width, int height,
                          int x, int y, const std::string &text) {
  int cursor_x = x;
  for (char c : text) {
    draw_char_bgr(data, stride, width, height, cursor_x, y, c);
    cursor_x += 6;
  }
}

static void draw_text_rgb(guint8 *data, int stride, int width, int height,
                          int x, int y, const std::string &text) {
  int cursor_x = x;
  for (char c : text) {
    draw_char_rgb(data, stride, width, height, cursor_x, y, c);
    cursor_x += 6;
  }
}

static void draw_text_rgba(guint8 *data, int stride, int width, int height,
                           int x, int y, const std::string &text) {
  int cursor_x = x;
  for (char c : text) {
    draw_char_rgba(data, stride, width, height, cursor_x, y, c);
    cursor_x += 6;
  }
}

static void draw_text_gray(guint8 *data, int stride, int width, int height,
                           int x, int y, const std::string &text) {
  int cursor_x = x;
  for (char c : text) {
    draw_char_gray(data, stride, width, height, cursor_x, y, c);
    cursor_x += 6;
  }
}

static void gst_ros2_overlay_class_init(GstRos2OverlayClass *klass) {
  GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
  GstElementClass *gstelement_class = GST_ELEMENT_CLASS(klass);
  GstBaseTransformClass *base_transform_class = GST_BASE_TRANSFORM_CLASS(klass);
  GstVideoFilterClass *video_filter_class = GST_VIDEO_FILTER_CLASS(klass);

  gobject_class->set_property = gst_ros2_overlay_set_property;
  gobject_class->get_property = gst_ros2_overlay_get_property;
  gobject_class->finalize = gst_ros2_overlay_finalize;

  g_object_class_install_property(
      gobject_class, PROP_NODE_NAME,
      g_param_spec_string(
          "node-name", "ROS 2 node name",
          "ROS 2 node name used by this element", DEFAULT_NODE_NAME,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
                        G_PARAM_CONSTRUCT_ONLY)));

  g_object_class_install_property(
      gobject_class, PROP_TEXT_TOPIC,
      g_param_spec_string("text-topic", "Text topic",
                          "ROS 2 std_msgs/msg/String topic", DEFAULT_TEXT_TOPIC,
                          (GParamFlags)(G_PARAM_READWRITE |
                                        G_PARAM_STATIC_STRINGS |
                                        G_PARAM_CONSTRUCT_ONLY)));

  g_object_class_install_property(
      gobject_class, PROP_DOT_TOPIC,
      g_param_spec_string(
          "dot-topic", "Dot topic", "ROS 2 geometry_msgs/msg/Vector3 topic",
          DEFAULT_DOT_TOPIC,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
                        G_PARAM_CONSTRUCT_ONLY)));

  g_object_class_install_property(
      gobject_class, PROP_QOS_PROFILE,
      g_param_spec_string(
          "qos-profile", "QoS profile",
          "QoS profile: 'default' or 'sensor_data'", DEFAULT_QOS_PROFILE,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
                        G_PARAM_CONSTRUCT_ONLY)));
  
  gst_ros2_common::install_owns_ros_property(gobject_class, PROP_OWNS_ROS);

  gst_element_class_set_static_metadata(
      gstelement_class, "ROS 2 Overlay", "Filter/Effect/Video",
      "Draws ROS 2 text and a ROS 2-controlled dot onto video frames",
      "Connor Needham (CPRT) <connor.needham2015@gmail.com>");

  GstCaps *caps = gst_caps_from_string(
      "video/x-raw, format=(string){ RGBA, RGB, BGR, GRAY8 }, "
      "width=(int)[1,MAX], height=(int)[1,MAX]");

  gst_element_class_add_pad_template(
      gstelement_class, gst_pad_template_new("src", GST_PAD_SRC, GST_PAD_ALWAYS,
                                             gst_caps_ref(caps)));
  gst_element_class_add_pad_template(
      gstelement_class,
      gst_pad_template_new("sink", GST_PAD_SINK, GST_PAD_ALWAYS, caps));
  gst_caps_unref(caps);

  base_transform_class->start = GST_DEBUG_FUNCPTR(gst_ros2_overlay_start);
  base_transform_class->stop = GST_DEBUG_FUNCPTR(gst_ros2_overlay_stop);

  video_filter_class->set_info = GST_DEBUG_FUNCPTR(gst_ros2_overlay_set_info);
  video_filter_class->transform_frame_ip =
      GST_DEBUG_FUNCPTR(gst_ros2_overlay_transform_frame_ip);

  GST_DEBUG_CATEGORY_INIT(gst_ros2_overlay_debug, "ros2overlay", 0,
                          "ROS2 Overlay");
}

static void gst_ros2_overlay_init(GstRos2Overlay *self) {
  self->node_name = g_strdup(DEFAULT_NODE_NAME);
  self->text_topic = g_strdup(DEFAULT_TEXT_TOPIC);
  self->dot_topic = g_strdup(DEFAULT_DOT_TOPIC);
  self->qos_profile = g_strdup(DEFAULT_QOS_PROFILE);
  self->owns_ros = DEFAULT_OWNS_ROS;

  self->priv = std::make_unique<cpp_ros2_overlay>();
  gst_video_info_init(&self->info);

  gst_base_transform_set_in_place(GST_BASE_TRANSFORM(self), TRUE);
}

static void gst_ros2_overlay_finalize(GObject *object) {
  GstRos2Overlay *self = GST_ROS2_OVERLAY(object);

  g_free(self->node_name);
  g_free(self->text_topic);
  g_free(self->dot_topic);
  g_free(self->qos_profile);

  G_OBJECT_CLASS(gst_ros2_overlay_parent_class)->finalize(object);
}

static void gst_ros2_overlay_set_property(GObject *object, guint prop_id,
                                          const GValue *value,
                                          GParamSpec *pspec) {
  GstRos2Overlay *self = GST_ROS2_OVERLAY(object);

  switch (prop_id) {
  case PROP_NODE_NAME:
    g_free(self->node_name);
    self->node_name = g_value_dup_string(value);
    break;
  case PROP_TEXT_TOPIC:
    g_free(self->text_topic);
    self->text_topic = g_value_dup_string(value);
    break;
  case PROP_DOT_TOPIC:
    g_free(self->dot_topic);
    self->dot_topic = g_value_dup_string(value);
    break;
  case PROP_QOS_PROFILE:
    g_free(self->qos_profile);
    self->qos_profile = g_value_dup_string(value);
    break;
  case PROP_OWNS_ROS:
    self->owns_ros = g_value_get_boolean(value);
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    break;
  }
}

static void gst_ros2_overlay_get_property(GObject *object, guint prop_id,
                                          GValue *value, GParamSpec *pspec) {
  GstRos2Overlay *self = GST_ROS2_OVERLAY(object);

  switch (prop_id) {
  case PROP_NODE_NAME:
    g_value_set_string(value, self->node_name);
    break;
  case PROP_TEXT_TOPIC:
    g_value_set_string(value, self->text_topic);
    break;
  case PROP_DOT_TOPIC:
    g_value_set_string(value, self->dot_topic);
    break;
  case PROP_QOS_PROFILE:
    g_value_set_string(value, self->qos_profile);
    break;
  case PROP_OWNS_ROS:
    g_value_set_boolean(value, self->owns_ros);
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    break;
  }
}

static gboolean gst_ros2_overlay_start(GstBaseTransform *trans) {
  GstRos2Overlay *self = GST_ROS2_OVERLAY(trans);

  if (!gst_ros2_common::acquire_ros(self->owns_ros)) {
    GST_ERROR_OBJECT(
        self,
        "ROS not initialized. Set owns-ros=true or initialize ROS externally.");
    return FALSE;
  }

  rclcpp::NodeOptions opts;
  self->priv->node = std::make_shared<rclcpp::Node>(
      self->node_name ? self->node_name : DEFAULT_NODE_NAME, opts);

  auto qos = gst_ros2_common::parse_qos_profile(
      self->qos_profile, DEFAULT_QOS_PROFILE);

  self->priv->text_sub =
      self->priv->node->create_subscription<std_msgs::msg::String>(
          self->text_topic ? self->text_topic : DEFAULT_TEXT_TOPIC, qos,
          [self](const std_msgs::msg::String::SharedPtr msg) {
            text_callback(self, msg);
          });

  self->priv->dot_sub =
      self->priv->node->create_subscription<geometry_msgs::msg::Vector3>(
          self->dot_topic ? self->dot_topic : DEFAULT_DOT_TOPIC, qos,
          [self](const geometry_msgs::msg::Vector3::SharedPtr msg) {
            dot_callback(self, msg);
          });

  self->priv->running.store(true);
  self->priv->ros_spin_thread = std::thread(ros_spin_thread_fn, self);

  return TRUE;
}

static gboolean gst_ros2_overlay_stop(GstBaseTransform *trans) {
  GstRos2Overlay *self = GST_ROS2_OVERLAY(trans);

  self->priv->running.store(false);

  if (self->priv->ros_spin_thread.joinable()) {
    self->priv->ros_spin_thread.join();
  }

  self->priv->text_sub.reset();
  self->priv->dot_sub.reset();
  self->priv->node.reset();

  gst_ros2_common::release_ros(self->owns_ros);

  return TRUE;
}

static gboolean gst_ros2_overlay_set_info(GstVideoFilter *filter,
                                          GstCaps *incaps, GstVideoInfo *ininfo,
                                          GstCaps *outcaps,
                                          GstVideoInfo *outinfo) {
  GstRos2Overlay *self = GST_ROS2_OVERLAY(filter);
  self->info = *ininfo;
  return TRUE;
}

static GstFlowReturn gst_ros2_overlay_transform_frame_ip(GstVideoFilter *filter,
                                                         GstVideoFrame *frame) {
  GstRos2Overlay *self = GST_ROS2_OVERLAY(filter);

  std::string text;
  bool have_text = false;
  double dot_x = 0.0;
  double dot_y = 0.0;
  bool have_dot = false;

  {
    std::lock_guard<std::mutex> lock(self->priv->data_mutex);
    text = self->priv->current_text;
    have_text = self->priv->have_text;
    dot_x = self->priv->dot_x;
    dot_y = self->priv->dot_y;
    have_dot = self->priv->have_dot;
  }

  auto *data = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame, 0));
  int stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
  int width = GST_VIDEO_FRAME_WIDTH(frame);
  int height = GST_VIDEO_FRAME_HEIGHT(frame);
  GstVideoFormat fmt = GST_VIDEO_FRAME_FORMAT(frame);

  if (fmt == GST_VIDEO_FORMAT_BGR) {
    if (have_text) {
      draw_text_bgr(data, stride, width, height, 20, 20, text);
    }
    if (have_dot) {
      draw_dot_bgr(data, stride, width, height, static_cast<int>(dot_x),
                   static_cast<int>(dot_y), 6);
    }
  } else if (fmt == GST_VIDEO_FORMAT_RGB) {
    if (have_text) {
      draw_text_rgb(data, stride, width, height, 20, 20, text);
    }
    if (have_dot) {
      draw_dot_rgb(data, stride, width, height, static_cast<int>(dot_x),
                   static_cast<int>(dot_y), 6);
    }
  } else if (fmt == GST_VIDEO_FORMAT_GRAY8) {
    if (have_text) {
      draw_text_gray(data, stride, width, height, 20, 20, text);
    }
    if (have_dot) {
      draw_dot_gray(data, stride, width, height, static_cast<int>(dot_x),
                    static_cast<int>(dot_y), 6);
    }
  } else if (fmt == GST_VIDEO_FORMAT_RGBA) {
    if (have_text) {
      draw_text_rgba(data, stride, width, height, 20, 20, text);
    }
    if (have_dot) {
      draw_dot_rgba(data, stride, width, height, static_cast<int>(dot_x),
                    static_cast<int>(dot_y), 6);
    }
  } else {
    GST_WARNING_OBJECT(self, "Unsupported format for drawing");
  }

  return GST_FLOW_OK;
}