#include "gstmorse_led.hpp"

#include "metric_plot.hpp"
#include "morse_decoder.hpp"
#include "morse_timing.hpp"

#include <algorithm>
#include <cstring>

GST_DEBUG_CATEGORY_STATIC(morse_led_debug);
#define GST_CAT_DEFAULT morse_led_debug

enum {
  PROP_0,
  PROP_START_DETECTION,
  PROP_ROI_X,
  PROP_ROI_Y,
  PROP_ROI_WIDTH,
  PROP_ROI_HEIGHT,
  PROP_WPM,
  PROP_CALIBRATE,
  PROP_CALIBRATION_SECONDS,
  PROP_DECODED_TEXT,
  PROP_LED_ON,
  PROP_DRAW_ROI,
  PROP_ON_MARGIN,
  PROP_MIN_TRANSITION_UNITS,
  PROP_GAP_DETECT_RATIO,
  PROP_DOT_MAX_UNITS,
  PROP_DASH_MIN_UNITS,
  PROP_METRIC,
  PROP_METRIC_PLOT_PATH,
};

enum {
  SIGNAL_CHARACTER_DECODED,
  SIGNAL_ROI_LOCKED,
  LAST_SIGNAL,
};

static guint morse_led_signals[LAST_SIGNAL] = {0};

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE(
    "sink", GST_PAD_SINK, GST_PAD_ALWAYS,
    GST_STATIC_CAPS("video/x-raw, format=(string)RGB, "
                    "framerate=(fraction)[0/1, 2147483647/1]"));

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE(
    "src", GST_PAD_SRC, GST_PAD_ALWAYS,
    GST_STATIC_CAPS("video/x-raw, format=(string)RGB, "
                    "framerate=(fraction)[0/1, 2147483647/1]"));

// Could add a text_src pad for text output if we want for the decoded text.
// static GstStaticPadTemplate text_src_template = GST_STATIC_PAD_TEMPLATE(
//     "text_src", GST_PAD_SRC, GST_PAD_ALWAYS,
//     GST_STATIC_CAPS("text/x-raw, format=(string)utf8"));

struct _GstMorseLED {
  GstVideoFilter parent;
  GMutex lock;
  gboolean start_detection;
  gint roi_x;
  gint roi_y;
  guint roi_width;
  guint roi_height;
  guint wpm;

  gboolean calibrate;
  gdouble calibration_seconds;
  gdouble calibration_start_sec;
  gboolean calibration_started;
  gdouble calibration_sum_x;
  gdouble calibration_sum_y;
  gdouble calibration_sum_weight;

  MorseDecoder *decoder;
  gboolean draw_roi;
  gboolean last_led_on;

  gfloat last_metric;
  MetricPlotRecorder *metric_plot;
};

static void gst_morse_led_init(GstMorseLED *self);
static void gst_morse_led_class_init(GstMorseLEDClass *klass);

static void gst_morse_led_finalize(GObject *object);
static void gst_morse_led_set_property(GObject *object, guint prop_id,
                                       const GValue *value, GParamSpec *pspec);
static void gst_morse_led_get_property(GObject *object, guint prop_id,
                                       GValue *value, GParamSpec *pspec);
static GstStateChangeReturn
gst_morse_led_change_state(GstElement *element, GstStateChange transition);

static GstFlowReturn gst_morse_led_transform_frame_ip(GstVideoFilter *filter,
                                                      GstVideoFrame *frame);

G_DEFINE_TYPE(GstMorseLED, gst_morse_led, GST_TYPE_VIDEO_FILTER)

static void gst_morse_led_reset_calibration(GstMorseLED *self) {
  self->calibration_started = FALSE;
  self->calibration_start_sec = 0.0;
  self->calibration_sum_x = 0.0;
  self->calibration_sum_y = 0.0;
  self->calibration_sum_weight = 0.0;
}

static void gst_morse_led_init(GstMorseLED *self) {
  g_mutex_init(&self->lock);
  self->start_detection = TRUE;
  self->roi_x = 0;
  self->roi_y = 0;
  self->roi_width = 96;
  self->roi_height = 96;
  self->wpm = MORSE_DEFAULT_WPM;
  self->calibrate = FALSE;
  self->calibration_seconds = 2.0;
  self->calibration_start_sec = 0.0;
  self->calibration_started = FALSE;
  self->calibration_sum_x = 0.0;
  self->calibration_sum_y = 0.0;
  self->calibration_sum_weight = 0.0;

  self->draw_roi = FALSE;
  self->last_led_on = FALSE;
  self->last_metric = 0.0f;
  self->metric_plot = new MetricPlotRecorder();

  self->decoder = morse_decoder_new();
  morse_decoder_set_wpm(self->decoder, self->wpm);
  if (self->start_detection) {
    morse_decoder_start(self->decoder);
  }
}

static void gst_morse_led_finalize(GObject *object) {
  GstMorseLED *self = GST_MORSE_LED(object);
  if (self->metric_plot != nullptr) {
    self->metric_plot->write_and_clear();
  }
  if (self->decoder != nullptr) {
    morse_decoder_free(self->decoder);
    self->decoder = nullptr;
  }
  delete self->metric_plot;
  self->metric_plot = nullptr;
  g_mutex_clear(&self->lock);
  G_OBJECT_CLASS(gst_morse_led_parent_class)->finalize(object);
}

static void gst_morse_led_set_property(GObject *object, guint prop_id,
                                       const GValue *value, GParamSpec *pspec) {
  GstMorseLED *self = GST_MORSE_LED(object);

  switch (prop_id) {
  case PROP_START_DETECTION:
    self->start_detection = g_value_get_boolean(value);
    if (self->start_detection) {
      morse_decoder_start(self->decoder);
    } else {
      morse_decoder_reset(self->decoder);
    }
    self->last_led_on = FALSE;
    break;
  case PROP_ROI_X: {
    const gint roi_x = g_value_get_int(value);
    if (roi_x < 0) {
      GST_WARNING_OBJECT(self, "Invalid ROI X (%d), skipping", roi_x);
      break;
    }
    g_mutex_lock(&self->lock);
    self->roi_x = roi_x;
    g_mutex_unlock(&self->lock);
    break;
  }
  case PROP_ROI_Y: {
    const gint roi_y = g_value_get_int(value);
    if (roi_y < 0) {
      GST_WARNING_OBJECT(self, "Invalid ROI Y (%d), skipping", roi_y);
      break;
    }
    g_mutex_lock(&self->lock);
    self->roi_y = roi_y;
    g_mutex_unlock(&self->lock);
    break;
  }
  case PROP_ROI_WIDTH:
    g_mutex_lock(&self->lock);
    self->roi_width = g_value_get_uint(value);
    g_mutex_unlock(&self->lock);
    break;
  case PROP_ROI_HEIGHT:
    g_mutex_lock(&self->lock);
    self->roi_height = g_value_get_uint(value);
    g_mutex_unlock(&self->lock);
    break;
  case PROP_WPM:
    self->wpm = g_value_get_uint(value);
    morse_decoder_set_wpm(self->decoder, self->wpm);
    break;
  case PROP_CALIBRATE:
    g_mutex_lock(&self->lock);
    self->calibrate = g_value_get_boolean(value);
    if (self->calibrate) {
      gst_morse_led_reset_calibration(self);
    }
    g_mutex_unlock(&self->lock);
    break;
  case PROP_CALIBRATION_SECONDS:
    self->calibration_seconds = g_value_get_double(value);
    break;
  case PROP_DRAW_ROI:
    self->draw_roi = g_value_get_boolean(value);
    break;
  case PROP_ON_MARGIN:
    g_mutex_lock(&self->lock);
    morse_decoder_set_on_margin(self->decoder, g_value_get_float(value));
    g_mutex_unlock(&self->lock);
    break;
  case PROP_MIN_TRANSITION_UNITS:
    g_mutex_lock(&self->lock);
    morse_decoder_set_min_transition_units(self->decoder,
                                           g_value_get_double(value));
    g_mutex_unlock(&self->lock);
    break;
  case PROP_GAP_DETECT_RATIO:
    morse_decoder_set_gap_detect_ratio(self->decoder,
                                       g_value_get_double(value));
    break;
  case PROP_DOT_MAX_UNITS:
    morse_decoder_set_dot_max_units(self->decoder, g_value_get_double(value));
    break;
  case PROP_DASH_MIN_UNITS:
    morse_decoder_set_dash_min_units(self->decoder, g_value_get_double(value));
    break;
  case PROP_METRIC_PLOT_PATH:
    if (self->metric_plot != nullptr) {
      self->metric_plot->set_path(g_value_get_string(value));
    }
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    break;
  }
}

static void gst_morse_led_get_property(GObject *object, guint prop_id,
                                       GValue *value, GParamSpec *pspec) {
  GstMorseLED *self = GST_MORSE_LED(object);

  switch (prop_id) {
  case PROP_START_DETECTION:
    g_value_set_boolean(value, self->start_detection);
    break;
  case PROP_ROI_X:
    g_mutex_lock(&self->lock);
    g_value_set_int(value, self->roi_x);
    g_mutex_unlock(&self->lock);
    break;
  case PROP_ROI_Y:
    g_mutex_lock(&self->lock);
    g_value_set_int(value, self->roi_y);
    g_mutex_unlock(&self->lock);
    break;
  case PROP_ROI_WIDTH:
    g_mutex_lock(&self->lock);
    g_value_set_uint(value, self->roi_width);
    g_mutex_unlock(&self->lock);
    break;
  case PROP_ROI_HEIGHT:
    g_mutex_lock(&self->lock);
    g_value_set_uint(value, self->roi_height);
    g_mutex_unlock(&self->lock);
    break;
  case PROP_WPM:
    g_value_set_uint(value, self->wpm);
    break;
  case PROP_CALIBRATE:
    g_mutex_lock(&self->lock);
    g_value_set_boolean(value, self->calibrate);
    g_mutex_unlock(&self->lock);
    break;
  case PROP_CALIBRATION_SECONDS:
    g_value_set_double(value, self->calibration_seconds);
    break;
  case PROP_DECODED_TEXT:
    g_mutex_lock(&self->lock);
    g_value_set_string(value, morse_decoder_get_text(self->decoder));
    g_mutex_unlock(&self->lock);
    break;
  case PROP_LED_ON:
    g_mutex_lock(&self->lock);
    g_value_set_boolean(value, morse_decoder_get_led_on(self->decoder));
    g_mutex_unlock(&self->lock);
    break;
  case PROP_DRAW_ROI:
    g_value_set_boolean(value, self->draw_roi);
    break;
  case PROP_ON_MARGIN:
    g_mutex_lock(&self->lock);
    g_value_set_float(value, morse_decoder_get_on_margin(self->decoder));
    g_mutex_unlock(&self->lock);
    break;
  case PROP_MIN_TRANSITION_UNITS:
    g_mutex_lock(&self->lock);
    g_value_set_double(value,
                       morse_decoder_get_min_transition_units(self->decoder));
    g_mutex_unlock(&self->lock);
    break;
  case PROP_GAP_DETECT_RATIO:
    g_value_set_double(value,
                       morse_decoder_get_gap_detect_ratio(self->decoder));
    break;
  case PROP_DOT_MAX_UNITS:
    g_value_set_double(value, morse_decoder_get_dot_max_units(self->decoder));
    break;
  case PROP_DASH_MIN_UNITS:
    g_value_set_double(value, morse_decoder_get_dash_min_units(self->decoder));
    break;
  case PROP_METRIC:
    g_value_set_float(value, self->last_metric);
    break;
  case PROP_METRIC_PLOT_PATH:
    g_value_set_string(value, self->metric_plot != nullptr
                                  ? self->metric_plot->path()
                                  : nullptr);
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    break;
  }
}

static void gst_morse_led_class_init(GstMorseLEDClass *klass) {
  GST_DEBUG_CATEGORY_INIT(morse_led_debug, "morseLED", 0, "Morse LED decoder");

  GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
  GstVideoFilterClass *vfilter_class = GST_VIDEO_FILTER_CLASS(klass);

  gobject_class->finalize = gst_morse_led_finalize;
  gobject_class->set_property = gst_morse_led_set_property;
  gobject_class->get_property = gst_morse_led_get_property;
  element_class->change_state = gst_morse_led_change_state;

  gst_element_class_set_static_metadata(
      element_class, "Morse LED Decoder", "Filter/Effect/Video",
      "Detects a blinking red LED and decodes Morse code",
      "Tomas Williston <tomaswilliston@gmail.com>");

  gst_element_class_add_pad_template(
      element_class, gst_static_pad_template_get(&sink_template));
  gst_element_class_add_pad_template(
      element_class, gst_static_pad_template_get(&src_template));
  // gst_element_class_add_pad_template(
  //     element_class, gst_static_pad_template_get(&text_src_template));

  vfilter_class->transform_frame_ip = gst_morse_led_transform_frame_ip;

  g_object_class_install_property(
      gobject_class, PROP_START_DETECTION,
      g_param_spec_boolean("start-detection", "Start Detection",
                           "Enable Morse decoding from incoming frames", TRUE,
                           (GParamFlags)(G_PARAM_READWRITE |
                                         GST_PARAM_MUTABLE_READY |
                                         G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_ROI_X,
      g_param_spec_int(
          "roi-x", "ROI X", "Top-left ROI x-coordinate", 0, 10000, 0,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_ROI_Y,
      g_param_spec_int(
          "roi-y", "ROI Y", "Top-left ROI y-coordinate", 0, 10000, 0,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_ROI_WIDTH,
      g_param_spec_uint(
          "roi-width", "ROI Width", "ROI width in pixels", 1, 10000, 96,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_ROI_HEIGHT,
      g_param_spec_uint(
          "roi-height", "ROI Height", "ROI height in pixels", 1, 10000, 96,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_WPM,
      g_param_spec_uint("wpm", "Words Per Minute",
                        "Morse speed in words per minute (PARIS timing)", 1,
                        200, MORSE_DEFAULT_WPM,
                        (GParamFlags)(G_PARAM_READWRITE |
                                      GST_PARAM_MUTABLE_READY |
                                      G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_CALIBRATE,
      g_param_spec_boolean(
          "calibrate", "Calibrate",
          "Find most-red region during calibration window and update ROI",
          FALSE, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_CALIBRATION_SECONDS,
      g_param_spec_double(
          "calibration-seconds", "Calibration Seconds",
          "Duration to gather red-dominance stats", 0.1, 30.0, 2.0,
          (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY |
                        G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_DECODED_TEXT,
      g_param_spec_string(
          "decoded-text", "Decoded Text",
          "Decoded Morse text (up to last 256 characters)", "",
          (GParamFlags)(G_PARAM_READABLE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_LED_ON,
      g_param_spec_boolean(
          "led-on", "LED On", "Current LED on/off decision", FALSE,
          (GParamFlags)(G_PARAM_READABLE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_DRAW_ROI,
      g_param_spec_boolean(
          "draw-roi", "Draw ROI",
          "Draw a bounding box around the detection ROI on output frames",
          FALSE,
          (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY |
                        G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_ON_MARGIN,
      g_param_spec_float(
          "on-margin", "On Margin",
          "Red-dominance margin above baseline to treat the LED as on ", 0.0f,
          1.0f, MORSE_DEFAULT_ON_MARGIN,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_MIN_TRANSITION_UNITS,
      g_param_spec_double(
          "min-transition-units", "Min Transition Units",
          "How long a candidate LED on/off state must persist (in dit units) "
          "before it is committed",
          0.0, 5.0, MORSE_DEFAULT_MIN_TRANSITION_UNITS,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_GAP_DETECT_RATIO,
      g_param_spec_double(
          "gap-detect-ratio", "Gap Detect Ratio",
          "Fraction of nominal letter/word gap used as the detection "
          "threshold",
          0.01, 1.0, MORSE_DEFAULT_GAP_DETECT_RATIO,
          (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY |
                        G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_DOT_MAX_UNITS,
      g_param_spec_double(
          "dot-max-units", "Dot Max Units",
          "Maximum ON duration in dit units classified as a dot", 0.0, 10.0,
          MORSE_DEFAULT_DOT_MAX_UNITS,
          (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY |
                        G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_DASH_MIN_UNITS,
      g_param_spec_double(
          "dash-min-units", "Dash Min Units",
          "Minimum ON duration in dit units classified as a dash", 0.0, 10.0,
          MORSE_DEFAULT_DASH_MIN_UNITS,
          (GParamFlags)(G_PARAM_READWRITE | GST_PARAM_MUTABLE_READY |
                        G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_METRIC,
      g_param_spec_float(
          "metric", "Metric",
          "Latest ROI lit-fraction metric fed to the Morse decoder", 0.0f, 1.0f,
          0.0f, (GParamFlags)(G_PARAM_READABLE | G_PARAM_STATIC_STRINGS)));
  g_object_class_install_property(
      gobject_class, PROP_METRIC_PLOT_PATH,
      g_param_spec_string(
          "metric-plot-path", "Metric Plot Path",
          "If set, write a metric-vs-time plot image (.ppm) and CSV when the "
          "element stops. For .png paths a .ppm/.csv with the same stem are "
          "written.",
          nullptr, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

  morse_led_signals[SIGNAL_CHARACTER_DECODED] =
      g_signal_new("character-decoded", G_TYPE_FROM_CLASS(klass),
                   static_cast<GSignalFlags>(G_SIGNAL_RUN_LAST), 0, nullptr,
                   nullptr, nullptr, G_TYPE_NONE, 1, G_TYPE_UINT);
  // Only used in test_morse_led.py right now, can be removed if not needed
  // later.
  morse_led_signals[SIGNAL_ROI_LOCKED] =
      g_signal_new("roi-locked", G_TYPE_FROM_CLASS(klass),
                   static_cast<GSignalFlags>(G_SIGNAL_RUN_LAST), 0, nullptr,
                   nullptr, nullptr, G_TYPE_NONE, 4, G_TYPE_INT, G_TYPE_INT,
                   G_TYPE_UINT, G_TYPE_UINT);
}

static gfloat gst_morse_led_pixel_dominance(const guint8 *px) {
  const gfloat r = static_cast<gfloat>(px[0]) / 255.0f;
  const gfloat g = static_cast<gfloat>(px[1]) / 255.0f;
  const gfloat b = static_cast<gfloat>(px[2]) / 255.0f;
  return std::max(0.0f, r - 0.5f * (g + b));
}

static void gst_morse_led_center_roi(GstMorseLED *self, gint frame_w,
                                     gint frame_h, gdouble center_x,
                                     gdouble center_y) {
  gint roi_x = static_cast<gint>(
      center_x - static_cast<gdouble>(self->roi_width) / 2.0 + 0.5);
  gint roi_y = static_cast<gint>(
      center_y - static_cast<gdouble>(self->roi_height) / 2.0 + 0.5);
  roi_x = std::max(
      0, std::min(roi_x, frame_w - static_cast<gint>(self->roi_width)));
  roi_y = std::max(
      0, std::min(roi_y, frame_h - static_cast<gint>(self->roi_height)));
  self->roi_x = roi_x;
  self->roi_y = roi_y;
}

static gfloat gst_morse_led_compute_roi_metric(const GstVideoFrame *frame,
                                               gint roi_x, gint roi_y,
                                               guint roi_w, guint roi_h) {
  const gint frame_w = GST_VIDEO_FRAME_WIDTH(frame);
  const gint frame_h = GST_VIDEO_FRAME_HEIGHT(frame);
  const gint x0 = std::max(0, std::min(roi_x, frame_w - 1));
  const gint y0 = std::max(0, std::min(roi_y, frame_h - 1));
  const gint x1 = std::min(frame_w, x0 + static_cast<gint>(roi_w));
  const gint y1 = std::min(frame_h, y0 + static_cast<gint>(roi_h));
  if (x1 <= x0 || y1 <= y0) {
    return 0.0f;
  }

  const guint8 *data =
      static_cast<const guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame, 0));
  const gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
  constexpr gfloat kDominanceGate = 0.3f;
  guint lit = 0;
  guint count = 0;
  for (gint y = y0; y < y1; ++y) {
    const guint8 *row = data + y * stride;
    for (gint x = x0; x < x1; ++x) {
      if (gst_morse_led_pixel_dominance(row + x * 3) > kDominanceGate) {
        ++lit;
      }
      ++count;
    }
  }
  if (count == 0) {
    return 0.0f;
  }
  return static_cast<gfloat>(lit) / static_cast<gfloat>(count);
}

// calibrates the ROI by accumulating the sum of the red-dominance of the
// pixels, ignores pixels with red-dominance <= 0.
static void gst_morse_led_accumulate_calibration(GstMorseLED *self,
                                                 const GstVideoFrame *frame) {
  const gint frame_w = GST_VIDEO_FRAME_WIDTH(frame);
  const gint frame_h = GST_VIDEO_FRAME_HEIGHT(frame);
  const guint8 *data =
      static_cast<const guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame, 0));
  const gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);

  gdouble sum_x = 0.0;
  gdouble sum_y = 0.0;
  gdouble sum_weight = 0.0;

  for (gint y = 0; y < frame_h; ++y) {
    const guint8 *row = data + y * stride;
    for (gint x = 0; x < frame_w; ++x) {
      const gfloat dominance = gst_morse_led_pixel_dominance(row + x * 3);
      if (dominance <= 0.0f) {
        continue;
      }
      sum_x += static_cast<gdouble>(x) * dominance;
      sum_y += static_cast<gdouble>(y) * dominance;
      sum_weight += dominance;
    }
  }

  self->calibration_sum_x += sum_x;
  self->calibration_sum_y += sum_y;
  self->calibration_sum_weight += sum_weight;
}

static void gst_morse_led_finish_calibration(GstMorseLED *self,
                                             const GstVideoFrame *frame) {
  if (self->calibration_sum_weight <= 0.0) {
    GST_WARNING("Calibration sum weight is 0, skipping calibration");
    return;
  }

  const gint frame_w = GST_VIDEO_FRAME_WIDTH(frame);
  const gint frame_h = GST_VIDEO_FRAME_HEIGHT(frame);
  const gdouble center_x =
      self->calibration_sum_x / self->calibration_sum_weight;
  const gdouble center_y =
      self->calibration_sum_y / self->calibration_sum_weight;
  gst_morse_led_center_roi(self, frame_w, frame_h, center_x, center_y);

  g_mutex_lock(&self->lock);
  gint roi_x = self->roi_x;
  gint roi_y = self->roi_y;
  const guint roi_width = self->roi_width;
  const guint roi_height = self->roi_height;
  g_mutex_unlock(&self->lock);

  g_object_notify(G_OBJECT(self), "roi-x");
  g_object_notify(G_OBJECT(self), "roi-y");
  g_signal_emit(self, morse_led_signals[SIGNAL_ROI_LOCKED], 0, roi_x, roi_y,
                roi_width, roi_height);
}

static void gst_morse_led_update_calibration(GstMorseLED *self,
                                             const GstVideoFrame *frame,
                                             gdouble timestamp_sec) {
  g_mutex_lock(&self->lock);
  if (!self->calibrate) {
    g_mutex_unlock(&self->lock);
    return;
  }
  g_mutex_unlock(&self->lock);
  if (!self->calibration_started) {
    self->calibration_started = TRUE;
    self->calibration_start_sec = timestamp_sec;
  }
  gst_morse_led_accumulate_calibration(self, frame);
  if (timestamp_sec - self->calibration_start_sec < self->calibration_seconds) {
    return;
  }
  gst_morse_led_finish_calibration(self, frame);
  g_mutex_lock(&self->lock);
  self->calibrate = FALSE;
  g_mutex_unlock(&self->lock);
  g_object_notify(G_OBJECT(self), "calibrate");
  gst_morse_led_reset_calibration(self);
}

static inline void gst_morse_led_set_pixel(guint8 *data, gint stride, gint x,
                                           gint y, guint8 r, guint8 g,
                                           guint8 b) {
  guint8 *px = data + y * stride + x * 3;
  px[0] = r;
  px[1] = g;
  px[2] = b;
}

static void gst_morse_led_draw_roi_box(GstVideoFrame *frame, gint roi_x,
                                       gint roi_y, guint roi_w, guint roi_h,
                                       gboolean led_on) {
  const gint frame_w = GST_VIDEO_FRAME_WIDTH(frame);
  const gint frame_h = GST_VIDEO_FRAME_HEIGHT(frame);
  if (frame_w <= 0 || frame_h <= 0) {
    GST_WARNING("Invalid frame size, skipping ROI box drawing");
    return;
  }
  const gint x0 = std::min(roi_x, frame_w - 1);
  const gint y0 = std::min(roi_y, frame_h - 1);
  const gint x1 = std::min(x0 + static_cast<gint>(roi_w) - 1, frame_w - 1);
  const gint y1 = std::min(y0 + static_cast<gint>(roi_h) - 1, frame_h - 1);
  if (x1 <= x0 || y1 <= y0) {
    GST_WARNING("Invalid ROI box coordinates, skipping ROI box drawing");
    return;
  }

  guint8 *data = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame, 0));
  const gint stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
  const guint8 r = led_on ? 0 : 255;
  const guint8 g = led_on ? 255 : 255;
  const guint8 b = 0;
  constexpr guint kThickness = 2;

  for (guint t = 0; t < kThickness; ++t) {
    const gint top = std::min(y1, y0 + static_cast<gint>(t));
    const gint bottom = std::max(y0, y1 - static_cast<gint>(t));
    const gint left = std::min(x1, x0 + static_cast<gint>(t));
    const gint right = std::max(x0, x1 - static_cast<gint>(t));

    for (gint x = left; x <= right; ++x) {
      gst_morse_led_set_pixel(data, stride, x, top, r, g, b);
      gst_morse_led_set_pixel(data, stride, x, bottom, r, g, b);
    }
    for (gint y = top; y <= bottom; ++y) {
      gst_morse_led_set_pixel(data, stride, left, y, r, g, b);
      gst_morse_led_set_pixel(data, stride, right, y, r, g, b);
    }
  }
}

static GstStateChangeReturn
gst_morse_led_change_state(GstElement *element, GstStateChange transition) {
  GstMorseLED *self = GST_MORSE_LED(element);
  if (transition == GST_STATE_CHANGE_READY_TO_NULL &&
      self->metric_plot != nullptr) {
    self->metric_plot->write_and_clear();
  }
  return GST_ELEMENT_CLASS(gst_morse_led_parent_class)
      ->change_state(element, transition);
}

static GstFlowReturn gst_morse_led_transform_frame_ip(GstVideoFilter *filter,
                                                      GstVideoFrame *frame) {
  GstMorseLED *self = GST_MORSE_LED(filter);

  const GstClockTime pts = GST_BUFFER_PTS(frame->buffer);
  gdouble timestamp_sec = 0.0;
  if (GST_CLOCK_TIME_IS_VALID(pts)) {
    timestamp_sec =
        static_cast<gdouble>(pts) / static_cast<gdouble>(GST_SECOND);
  } else {
    GST_ERROR("Invalid PTS, skipping frame");
    return GST_FLOW_ERROR;
  }

  g_mutex_lock(&self->lock);
  gint roi_x = self->roi_x;
  gint roi_y = self->roi_y;
  guint roi_width = self->roi_width;
  guint roi_height = self->roi_height;
  g_mutex_unlock(&self->lock);

  if (self->start_detection) {
    gst_morse_led_update_calibration(self, frame, timestamp_sec);

    g_mutex_lock(&self->lock);
    roi_x = self->roi_x;
    roi_y = self->roi_y;
    g_mutex_unlock(&self->lock);

    gfloat metric = gst_morse_led_compute_roi_metric(frame, roi_x, roi_y,
                                                     roi_width, roi_height);
    while (metric >= 1.0f) {
      g_mutex_lock(&self->lock);
      self->roi_x = std::max(1, self->roi_x - 50);
      self->roi_y = std::max(1, self->roi_y - 50);
      self->roi_width = static_cast<guint>(
          std::min(GST_VIDEO_FRAME_WIDTH(frame) - 1,
                   static_cast<gint>(self->roi_width + 100)));
      self->roi_height = static_cast<guint>(
          std::min(GST_VIDEO_FRAME_HEIGHT(frame) - 1,
                   static_cast<gint>(self->roi_height + 100)));
      roi_x = self->roi_x;
      roi_y = self->roi_y;
      roi_width = self->roi_width;
      roi_height = self->roi_height;
      g_mutex_unlock(&self->lock);
      metric = gst_morse_led_compute_roi_metric(frame, roi_x, roi_y, roi_width,
                                                roi_height);
    }

    gchar new_chars[16];
    gsize new_char_count = 0;
    g_mutex_lock(&self->lock);
    const gchar *old_text = morse_decoder_get_text(self->decoder);
    const gsize old_len = std::strlen(old_text);
    morse_decoder_process_sample(self->decoder, metric, timestamp_sec);
    const gchar *new_text = morse_decoder_get_text(self->decoder);
    const gsize new_len = std::strlen(new_text);
    if (new_len > old_len) {
      new_char_count = std::min(new_len - old_len, sizeof(new_chars));
      std::memcpy(new_chars, new_text + old_len, new_char_count);
    }
    const gboolean led_on_now = morse_decoder_get_led_on(self->decoder);
    const gfloat on_margin = morse_decoder_get_on_margin(self->decoder);
    const gfloat baseline = morse_decoder_get_baseline(self->decoder);
    const gfloat threshold_on = baseline + on_margin;
    const gfloat threshold_off = baseline + on_margin * 0.5f;
    self->last_metric = metric;
    if (self->metric_plot != nullptr) {
      self->metric_plot->append(timestamp_sec, metric, threshold_on,
                                threshold_off, led_on_now);
    }
    g_mutex_unlock(&self->lock);
    if (new_char_count > 0) {
      for (gsize i = 0; i < new_char_count; ++i) {
        g_signal_emit(self, morse_led_signals[SIGNAL_CHARACTER_DECODED], 0,
                      static_cast<guint>(static_cast<guchar>(new_chars[i])));
      }
      g_object_notify(G_OBJECT(self), "decoded-text");
    }

    const gboolean led_on = morse_decoder_get_led_on(self->decoder);
    if (led_on != self->last_led_on) {
      self->last_led_on = led_on;
      g_object_notify(G_OBJECT(self), "led-on");
    }
  }

  if (self->draw_roi) {
    const gboolean led_on =
        self->start_detection ? morse_decoder_get_led_on(self->decoder) : FALSE;
    gst_morse_led_draw_roi_box(frame, roi_x, roi_y, roi_width, roi_height,
                               led_on);
  }

  return GST_FLOW_OK;
}
