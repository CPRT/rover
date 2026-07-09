#include "gstmorse_led.hpp"

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
};

static void gst_morse_led_init(GstMorseLED *self);
static void gst_morse_led_class_init(GstMorseLEDClass *klass);

static void gst_morse_led_finalize(GObject *object);
static void gst_morse_led_set_property(GObject *object, guint prop_id,
                                       const GValue *value, GParamSpec *pspec);
static void gst_morse_led_get_property(GObject *object, guint prop_id,
                                       GValue *value, GParamSpec *pspec);

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

  self->decoder = morse_decoder_new();
  morse_decoder_set_wpm(self->decoder, self->wpm);
  if (self->start_detection) {
    morse_decoder_start(self->decoder);
  }
}

static void gst_morse_led_finalize(GObject *object) {
  GstMorseLED *self = GST_MORSE_LED(object);
  if (self->decoder != nullptr) {
    morse_decoder_free(self->decoder);
    self->decoder = nullptr;
  }
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
  case PROP_ROI_X:
    self->roi_x = g_value_get_int(value);
    break;
  case PROP_ROI_Y:
    self->roi_y = g_value_get_int(value);
    break;
  case PROP_ROI_WIDTH:
    self->roi_width = g_value_get_uint(value);
    break;
  case PROP_ROI_HEIGHT:
    self->roi_height = g_value_get_uint(value);
    break;
  case PROP_WPM:
    self->wpm = g_value_get_uint(value);
    morse_decoder_set_wpm(self->decoder, self->wpm);
    break;
  case PROP_CALIBRATE:
    self->calibrate = g_value_get_boolean(value);
    if (self->calibrate) {
      gst_morse_led_reset_calibration(self);
    }
    break;
  case PROP_CALIBRATION_SECONDS:
    self->calibration_seconds = g_value_get_double(value);
    break;
  case PROP_DRAW_ROI:
    self->draw_roi = g_value_get_boolean(value);
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
    g_value_set_int(value, self->roi_x);
    break;
  case PROP_ROI_Y:
    g_value_set_int(value, self->roi_y);
    break;
  case PROP_ROI_WIDTH:
    g_value_set_uint(value, self->roi_width);
    break;
  case PROP_ROI_HEIGHT:
    g_value_set_uint(value, self->roi_height);
    break;
  case PROP_WPM:
    g_value_set_uint(value, self->wpm);
    break;
  case PROP_CALIBRATE:
    g_value_set_boolean(value, self->calibrate);
    break;
  case PROP_CALIBRATION_SECONDS:
    g_value_set_double(value, self->calibration_seconds);
    break;
  case PROP_DECODED_TEXT:
    g_value_set_string(value, morse_decoder_get_text(self->decoder));
    break;
  case PROP_LED_ON:
    g_value_set_boolean(value, morse_decoder_get_led_on(self->decoder));
    break;
  case PROP_DRAW_ROI:
    g_value_set_boolean(value, self->draw_roi);
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
      g_param_spec_boolean(
          "start-detection", "Start Detection",
          "Enable Morse decoding from incoming frames", TRUE,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
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
      g_param_spec_uint(
          "wpm", "Words Per Minute",
          "Morse speed in words per minute (PARIS timing)", 1, 200,
          MORSE_DEFAULT_WPM,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
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
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));
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
          FALSE, (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS)));

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
  gdouble sum = 0.0;
  guint count = 0;
  for (gint y = y0; y < y1; ++y) {
    const guint8 *row = data + y * stride;
    for (gint x = x0; x < x1; ++x) {
      sum += gst_morse_led_pixel_dominance(row + x * 3);
      ++count;
    }
  }
  if (count == 0) {
    return 0.0f;
  }
  return static_cast<gfloat>(sum / static_cast<gdouble>(count));
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

  for (gint y = 0; y < frame_h; ++y) {
    const guint8 *row = data + y * stride;
    for (gint x = 0; x < frame_w; ++x) {
      const gfloat dominance = gst_morse_led_pixel_dominance(row + x * 3);
      if (dominance <= 0.0f) {
        continue;
      }
      self->calibration_sum_x += static_cast<gdouble>(x) * dominance;
      self->calibration_sum_y += static_cast<gdouble>(y) * dominance;
      self->calibration_sum_weight += dominance;
    }
  }
}

static void gst_morse_led_finish_calibration(GstMorseLED *self,
                                             const GstVideoFrame *frame) {
  if (self->calibration_sum_weight <= 0.0) {
    return;
  }

  const gint frame_w = GST_VIDEO_FRAME_WIDTH(frame);
  const gint frame_h = GST_VIDEO_FRAME_HEIGHT(frame);
  const gdouble center_x =
      self->calibration_sum_x / self->calibration_sum_weight;
  const gdouble center_y =
      self->calibration_sum_y / self->calibration_sum_weight;
  gst_morse_led_center_roi(self, frame_w, frame_h, center_x, center_y);

  g_object_notify(G_OBJECT(self), "roi-x");
  g_object_notify(G_OBJECT(self), "roi-y");
  g_signal_emit(self, morse_led_signals[SIGNAL_ROI_LOCKED], 0, self->roi_x,
                self->roi_y, self->roi_width, self->roi_height);
}

static void gst_morse_led_update_calibration(GstMorseLED *self,
                                             const GstVideoFrame *frame,
                                             gdouble timestamp_sec) {
  if (!self->calibrate) {
    return;
  }
  if (!self->calibration_started) {
    self->calibration_started = TRUE;
    self->calibration_start_sec = timestamp_sec;
  }

  gst_morse_led_accumulate_calibration(self, frame);

  if (timestamp_sec - self->calibration_start_sec < self->calibration_seconds) {
    return;
  }

  gst_morse_led_finish_calibration(self, frame);
  self->calibrate = FALSE;
  g_object_notify(G_OBJECT(self), "calibrate");
  gst_morse_led_reset_calibration(self);
}

static void gst_morse_led_set_pixel(guint8 *data, gint stride, gint x, gint y,
                                    guint8 r, guint8 g, guint8 b) {
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
  const gint x0 = std::max(0, std::min(roi_x, frame_w - 1));
  const gint y0 = std::max(0, std::min(roi_y, frame_h - 1));
  const gint x1 = std::min(frame_w - 1, x0 + static_cast<gint>(roi_w) - 1);
  const gint y1 = std::min(frame_h - 1, y0 + static_cast<gint>(roi_h) - 1);
  if (x1 <= x0 || y1 <= y0) {
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

    for (gint x = x0; x <= x1; ++x) {
      gst_morse_led_set_pixel(data, stride, x, top, r, g, b);
      gst_morse_led_set_pixel(data, stride, x, bottom, r, g, b);
    }
    for (gint y = y0; y <= y1; ++y) {
      gst_morse_led_set_pixel(data, stride, left, y, r, g, b);
      gst_morse_led_set_pixel(data, stride, right, y, r, g, b);
    }
  }
}

static GstFlowReturn gst_morse_led_transform_frame_ip(GstVideoFilter *filter,
                                                      GstVideoFrame *frame) {
  GstMorseLED *self = GST_MORSE_LED(filter);

  const GstClockTime pts = GST_BUFFER_PTS(frame->buffer);
  gdouble timestamp_sec = 0.0;
  if (GST_CLOCK_TIME_IS_VALID(pts)) {
    timestamp_sec =
        static_cast<gdouble>(pts) / static_cast<gdouble>(GST_SECOND);
  }

  if (self->start_detection) {
    gst_morse_led_update_calibration(self, frame, timestamp_sec);

    const gfloat metric = gst_morse_led_compute_roi_metric(
        frame, self->roi_x, self->roi_y, self->roi_width, self->roi_height);

    const gchar *old_text = morse_decoder_get_text(self->decoder);
    const gsize old_len = std::strlen(old_text);
    morse_decoder_process_sample(self->decoder, metric, timestamp_sec);
    const gchar *new_text = morse_decoder_get_text(self->decoder);
    const gsize new_len = std::strlen(new_text);
    if (new_len > old_len) {
      for (gsize i = old_len; i < new_len; ++i) {
        g_signal_emit(self, morse_led_signals[SIGNAL_CHARACTER_DECODED], 0,
                      static_cast<guint>(static_cast<guchar>(new_text[i])));
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
    gst_morse_led_draw_roi_box(frame, self->roi_x, self->roi_y, self->roi_width,
                               self->roi_height, led_on);
  }

  return GST_FLOW_OK;
}
