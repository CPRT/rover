#include "gstarucomarker.hpp"

struct _GstArucoMarker {
  GstVideoFilter parent;
  cv::Ptr<cv::aruco::Dictionary> dictionary;
  std::vector<int> ids;
  std::vector<std::vector<cv::Point2f>> corners;
  guint frame_count;
  guint detect_every;     // Detect markers every N frames
  gboolean draw_markers;  // Whether to draw detected markers
};

static void gst_arucomarker_init(GstArucoMarker *self) {
  self->dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
  self->frame_count = 0;
  self->detect_every = 1;
  self->ids = std::vector<int>();
  self->corners = std::vector<std::vector<cv::Point2f>>();
  self->draw_markers = TRUE;
}

G_DEFINE_TYPE(GstArucoMarker, gst_arucomarker, GST_TYPE_VIDEO_FILTER)

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE(
    "sink", GST_PAD_SINK, GST_PAD_ALWAYS,
    GST_STATIC_CAPS("video/x-raw, format=(string)BGR, "
                    "framerate=(fraction)[0/1, 2147483647/1]"));

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE(
    "src", GST_PAD_SRC, GST_PAD_ALWAYS,
    GST_STATIC_CAPS("video/x-raw, format=(string)BGR, "
                    "framerate=(fraction)[0/1, 2147483647/1]"));

static GstFlowReturn gst_arucomarker_transform_frame(GstVideoFilter *filter,
                                                     GstVideoFrame *inframe,
                                                     GstVideoFrame *outframe) {
  GstArucoMarker *self = GST_ARUCOMARKER(filter);
  self->frame_count++;

  cv::Mat frame(inframe->info.height, inframe->info.width, CV_8UC3,
                GST_VIDEO_FRAME_PLANE_DATA(inframe, 0));
  auto &ids = self->ids;
  auto &corners = self->corners;

  if (self->frame_count % self->detect_every == 0) {
    ids.clear();
    corners.clear();
    cv::aruco::detectMarkers(frame, self->dictionary, corners, ids);
    for (const int &id : ids) {
      g_signal_emit_by_name(self, "marker-detected", id);
    }
  }
  if (!ids.empty() && self->draw_markers) {
    cv::aruco::drawDetectedMarkers(frame, corners, ids);
  }
  frame.copyTo(cv::Mat(outframe->info.height, outframe->info.width, CV_8UC3,
                       GST_VIDEO_FRAME_PLANE_DATA(outframe, 0)));
  return GST_FLOW_OK;
}

static void gst_arucomarker_set_property(GObject *object, guint prop_id,
                                         const GValue *value,
                                         GParamSpec *pspec) {
  GstArucoMarker *self = GST_ARUCOMARKER(object);
  switch (prop_id) {
    case 1:
      self->detect_every = g_value_get_uint(value);
      break;
    case 2:
      self->draw_markers = g_value_get_boolean(value);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void gst_arucomarker_get_property(GObject *object, guint prop_id,
                                         GValue *value, GParamSpec *pspec) {
  GstArucoMarker *self = GST_ARUCOMARKER(object);
  switch (prop_id) {
    case 1:
      g_value_set_uint(value, self->detect_every);
      break;
    case 2:
      g_value_set_boolean(value, self->draw_markers);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
      break;
  }
}

static void gst_arucomarker_class_init(GstArucoMarkerClass *klass) {
  GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
  gobject_class->set_property = gst_arucomarker_set_property;
  gobject_class->get_property = gst_arucomarker_get_property;

  GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
  GstVideoFilterClass *vfilter_class = GST_VIDEO_FILTER_CLASS(klass);

  gst_element_class_set_static_metadata(
      element_class, "Aruco Marker Detection Filter", "Filter/Effect/Video",
      "Detects ArUco tags in video frames and emits marker ID",
      "Connor Needham <connor.needham2015@gmail.com>");

  gst_element_class_add_pad_template(
      element_class, gst_static_pad_template_get(&sink_template));
  gst_element_class_add_pad_template(
      element_class, gst_static_pad_template_get(&src_template));

  vfilter_class->transform_frame = gst_arucomarker_transform_frame;

  g_signal_new("marker-detected", G_TYPE_FROM_CLASS(klass), G_SIGNAL_RUN_LAST,
               0, nullptr, nullptr, g_cclosure_marshal_VOID__INT, G_TYPE_NONE,
               1, G_TYPE_INT);
  g_object_class_install_property(
      G_OBJECT_CLASS(klass), 1,
      g_param_spec_uint(
          "detect-every", "Detect Every",
          "Run detection every N frames (1 = every frame)", 1, G_MAXUINT, 1,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      G_OBJECT_CLASS(klass), 2,
      g_param_spec_boolean(
          "draw-markers", "Draw Markers",
          "Whether to draw detected markers on the frame", TRUE,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
}
static gboolean plugin_init(GstPlugin *plugin) {
  return gst_element_register(plugin, "arucomarker", GST_RANK_NONE,
                              GST_TYPE_ARUCOMARKER);
}

GST_PLUGIN_DEFINE(GST_VERSION_MAJOR, GST_VERSION_MINOR, arucomarker,
                  "Aruco marker detection filter", plugin_init, "1.0", "LGPL",
                  "arucomarker", "https://your-project-url.org")
