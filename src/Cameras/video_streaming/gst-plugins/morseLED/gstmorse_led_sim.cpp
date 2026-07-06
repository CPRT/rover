#include "gstmorse_led_sim.hpp"

#include <cmath>
#include <cstring>
#include <string>
#include <vector>

GST_DEBUG_CATEGORY_STATIC(morse_led_sim_debug);
#define GST_CAT_DEFAULT morse_led_sim_debug

enum {
  PROP_0,
  PROP_MESSAGE,
  PROP_WPM,
  PROP_WIDTH,
  PROP_HEIGHT,
  PROP_FRAMERATE,
  PROP_DOT_X,
  PROP_DOT_Y,
  PROP_DOT_RADIUS,
  PROP_LEAD_IN,
  PROP_LOOP,
};

#define DEFAULT_MESSAGE "SOS"
#define DEFAULT_WPM 20
#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 480
#define DEFAULT_FRAMERATE_NUM 30
#define DEFAULT_FRAMERATE_DEN 1
#define DEFAULT_DOT_RADIUS 12
#define DEFAULT_LEAD_IN 1.0
#define DEFAULT_LOOP TRUE

struct TimelineSegment {
  gdouble duration;
  gboolean led_on;
};

struct _GstMorseLEDSim {
  GstPushSrc parent;

  gchar *message;
  guint wpm;
  guint width;
  guint height;
  gint framerate_num;
  gint framerate_den;
  gint dot_x;
  gint dot_y;
  guint dot_radius;
  gdouble lead_in;
  gboolean loop;

  GstVideoInfo vinfo;
  gdouble frame_duration;
  gdouble total_duration;
  std::vector<TimelineSegment> timeline;
  guint64 frame_index;
};

static const char *morse_symbol_for_char(char c) {
  switch (c) {
  case 'A':
    return ".-";
  case 'B':
    return "-...";
  case 'C':
    return "-.-.";
  case 'D':
    return "-..";
  case 'E':
    return ".";
  case 'F':
    return "..-.";
  case 'G':
    return "--.";
  case 'H':
    return "....";
  case 'I':
    return "..";
  case 'J':
    return ".---";
  case 'K':
    return "-.-";
  case 'L':
    return ".-..";
  case 'M':
    return "--";
  case 'N':
    return "-.";
  case 'O':
    return "---";
  case 'P':
    return ".--.";
  case 'Q':
    return "--.-";
  case 'R':
    return ".-.";
  case 'S':
    return "...";
  case 'T':
    return "-";
  case 'U':
    return "..-";
  case 'V':
    return "...-";
  case 'W':
    return ".--";
  case 'X':
    return "-..-";
  case 'Y':
    return "-.--";
  case 'Z':
    return "--..";
  case '0':
    return "-----";
  case '1':
    return ".----";
  case '2':
    return "..---";
  case '3':
    return "...--";
  case '4':
    return "....-";
  case '5':
    return ".....";
  case '6':
    return "-....";
  case '7':
    return "--...";
  case '8':
    return "---..";
  case '9':
    return "----.";
  default:
    return nullptr;
  }
}

static void append_off(std::vector<TimelineSegment> &timeline, gdouble duration) {
  if (duration <= 0.0) {
    return;
  }
  if (!timeline.empty() && !timeline.back().led_on) {
    timeline.back().duration += duration;
    return;
  }
  timeline.push_back({duration, FALSE});
}

static void append_on(std::vector<TimelineSegment> &timeline, gdouble duration) {
  if (duration <= 0.0) {
    return;
  }
  timeline.push_back({duration, TRUE});
}

static std::vector<TimelineSegment>
build_timeline(const gchar *message, guint wpm) {
  std::vector<TimelineSegment> timeline;
  if (message == nullptr || message[0] == '\0' || wpm == 0) {
    return timeline;
  }

  const gdouble unit = 1.2 / static_cast<gdouble>(wpm);
  std::string normalized;
  for (const char *p = message; *p != '\0'; ++p) {
    normalized.push_back(static_cast<char>(g_ascii_toupper(*p)));
  }

  std::vector<std::string> words;
  std::string current;
  for (char c : normalized) {
    if (c == ' ') {
      if (!current.empty()) {
        words.push_back(current);
        current.clear();
      }
    } else {
      current.push_back(c);
    }
  }
  if (!current.empty()) {
    words.push_back(current);
  }

  for (size_t word_idx = 0; word_idx < words.size(); ++word_idx) {
    const std::string &word = words[word_idx];
    for (size_t letter_idx = 0; letter_idx < word.size(); ++letter_idx) {
      const char *symbols = morse_symbol_for_char(word[letter_idx]);
      if (symbols == nullptr) {
        continue;
      }

      for (size_t sym_idx = 0; symbols[sym_idx] != '\0'; ++sym_idx) {
        const gdouble on_duration =
            symbols[sym_idx] == '.' ? unit : 3.0 * unit;
        append_on(timeline, on_duration);

        const bool has_next_symbol = symbols[sym_idx + 1] != '\0';
        const bool has_next_letter = letter_idx + 1 < word.size();
        const bool has_next_word = word_idx + 1 < words.size();

        gdouble gap = 0.0;
        if (has_next_symbol) {
          gap = unit;
        } else if (has_next_letter) {
          gap = 3.0 * unit;
        } else if (has_next_word) {
          gap = 7.0 * unit;
        }
        append_off(timeline, gap);
      }
    }
  }

  return timeline;
}

static gdouble timeline_duration(const std::vector<TimelineSegment> &timeline) {
  gdouble total = 0.0;
  for (const TimelineSegment &segment : timeline) {
    total += segment.duration;
  }
  return total;
}

static gboolean timeline_led_on_at(const std::vector<TimelineSegment> &timeline,
                                   gdouble time, gdouble total_duration,
                                   gboolean loop) {
  if (timeline.empty() || time < 0.0) {
    return FALSE;
  }

  if (loop) {
    time = std::fmod(time, total_duration);
  } else if (time >= total_duration) {
    return FALSE;
  }

  gdouble elapsed = 0.0;
  for (const TimelineSegment &segment : timeline) {
    if (time < elapsed + segment.duration) {
      return segment.led_on;
    }
    elapsed += segment.duration;
  }
  return FALSE;
}

static void gst_morse_led_sim_rebuild_timeline(GstMorseLEDSim *self) {
  self->timeline = build_timeline(self->message, self->wpm);
  self->total_duration = timeline_duration(self->timeline);
}

static void gst_morse_led_sim_fill_black(GstVideoFrame *frame) {
  auto *data = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame, 0));
  const gsize stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
  const guint height = GST_VIDEO_FRAME_HEIGHT(frame);

  for (guint y = 0; y < height; ++y) {
    std::memset(data + y * stride, 0, stride);
  }
}

static void gst_morse_led_sim_draw_dot(GstVideoFrame *frame, gint cx, gint cy,
                                       guint radius) {
  auto *data = static_cast<guint8 *>(GST_VIDEO_FRAME_PLANE_DATA(frame, 0));
  const gsize stride = GST_VIDEO_FRAME_PLANE_STRIDE(frame, 0);
  const guint width = GST_VIDEO_FRAME_WIDTH(frame);
  const guint height = GST_VIDEO_FRAME_HEIGHT(frame);
  const gint radius_sq = static_cast<gint>(radius * radius);

  const gint y_min = std::max(0, cy - static_cast<gint>(radius));
  const gint y_max =
      std::min(static_cast<gint>(height) - 1, cy + static_cast<gint>(radius));
  const gint x_min = std::max(0, cx - static_cast<gint>(radius));
  const gint x_max =
      std::min(static_cast<gint>(width) - 1, cx + static_cast<gint>(radius));

  for (gint y = y_min; y <= y_max; ++y) {
    for (gint x = x_min; x <= x_max; ++x) {
      const gint dx = x - cx;
      const gint dy = y - cy;
      if (dx * dx + dy * dy > radius_sq) {
        continue;
      }
      guint8 *pixel = data + y * stride + x * 3;
      pixel[0] = 255;
      pixel[1] = 0;
      pixel[2] = 0;
    }
  }
}

static void gst_morse_led_sim_set_property(GObject *object, guint prop_id,
                                           const GValue *value,
                                           GParamSpec *pspec);
static void gst_morse_led_sim_get_property(GObject *object, guint prop_id,
                                           GValue *value, GParamSpec *pspec);
static void gst_morse_led_sim_finalize(GObject *object);
static gboolean gst_morse_led_sim_start(GstBaseSrc *src);
static gboolean gst_morse_led_sim_stop(GstBaseSrc *src);
static GstFlowReturn gst_morse_led_sim_create(GstPushSrc *src,
                                              GstBuffer **buffer);

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE(
    "src", GST_PAD_SRC, GST_PAD_ALWAYS,
    GST_STATIC_CAPS("video/x-raw, format=(string)RGB, "
                    "width=(int)[1,4096], height=(int)[1,4096], "
                    "framerate=(fraction)[1/1,240/1]"));

G_DEFINE_TYPE(GstMorseLEDSim, gst_morse_led_sim, GST_TYPE_PUSH_SRC)

static void gst_morse_led_sim_class_init(GstMorseLEDSimClass *klass) {
  GST_DEBUG_CATEGORY_INIT(morse_led_sim_debug, "morseLEDSim", 0,
                          "Morse LED test source");

  GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
  GstBaseSrcClass *basesrc_class = GST_BASE_SRC_CLASS(klass);
  GstPushSrcClass *pushsrc_class = GST_PUSH_SRC_CLASS(klass);

  gobject_class->set_property = gst_morse_led_sim_set_property;
  gobject_class->get_property = gst_morse_led_sim_get_property;
  gobject_class->finalize = gst_morse_led_sim_finalize;

  g_object_class_install_property(
      gobject_class, PROP_MESSAGE,
      g_param_spec_string("message", "Message",
                          "Morse message to transmit (A-Z, 0-9, spaces)",
                          DEFAULT_MESSAGE,
                          (GParamFlags)(G_PARAM_READWRITE |
                                        G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_WPM,
      g_param_spec_uint("wpm", "Words Per Minute",
                        "Morse speed using PARIS timing (1 unit = 1.2/wpm s)",
                        1, 200, DEFAULT_WPM,
                        (GParamFlags)(G_PARAM_READWRITE |
                                      G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_WIDTH,
      g_param_spec_uint("width", "Width", "Frame width in pixels", 16, 4096,
                        DEFAULT_WIDTH,
                        (GParamFlags)(G_PARAM_READWRITE |
                                      G_PARAM_STATIC_STRINGS |
                                      G_PARAM_CONSTRUCT_ONLY)));

  g_object_class_install_property(
      gobject_class, PROP_HEIGHT,
      g_param_spec_uint("height", "Height", "Frame height in pixels", 16, 4096,
                        DEFAULT_HEIGHT,
                        (GParamFlags)(G_PARAM_READWRITE |
                                      G_PARAM_STATIC_STRINGS |
                                      G_PARAM_CONSTRUCT_ONLY)));

  g_object_class_install_property(
      gobject_class, PROP_FRAMERATE,
      gst_param_spec_fraction(
          "framerate", "Frame Rate", "Output framerate", 1, 1, 240, 1,
          DEFAULT_FRAMERATE_NUM, DEFAULT_FRAMERATE_DEN,
          (GParamFlags)(G_PARAM_READWRITE | G_PARAM_STATIC_STRINGS |
                        G_PARAM_CONSTRUCT_ONLY)));

  g_object_class_install_property(
      gobject_class, PROP_DOT_X,
      g_param_spec_int("dot-x", "Dot X", "LED center X in pixels (-1 = center)",
                       -1, 4096, -1,
                       (GParamFlags)(G_PARAM_READWRITE |
                                     G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_DOT_Y,
      g_param_spec_int("dot-y", "Dot Y", "LED center Y in pixels (-1 = center)",
                       -1, 4096, -1,
                       (GParamFlags)(G_PARAM_READWRITE |
                                     G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_DOT_RADIUS,
      g_param_spec_uint("dot-radius", "Dot Radius",
                        "LED radius in pixels", 1, 256, DEFAULT_DOT_RADIUS,
                        (GParamFlags)(G_PARAM_READWRITE |
                                      G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_LEAD_IN,
      g_param_spec_double("lead-in", "Lead In",
                          "Seconds of black frames before the message starts",
                          0.0, 60.0, DEFAULT_LEAD_IN,
                          (GParamFlags)(G_PARAM_READWRITE |
                                        G_PARAM_STATIC_STRINGS)));

  g_object_class_install_property(
      gobject_class, PROP_LOOP,
      g_param_spec_boolean("loop", "Loop",
                           "Repeat the message after it finishes", DEFAULT_LOOP,
                           (GParamFlags)(G_PARAM_READWRITE |
                                         G_PARAM_STATIC_STRINGS)));

  gst_element_class_set_static_metadata(
      element_class, "Morse LED Simulator", "Source/Video",
      "Generates a black video with a blinking red dot sending Morse code",
      "Tomas Williston <tomaswilliston@gmail.com>");

  gst_element_class_add_pad_template(
      element_class, gst_static_pad_template_get(&src_template));

  basesrc_class->start = gst_morse_led_sim_start;
  basesrc_class->stop = gst_morse_led_sim_stop;
  pushsrc_class->create = gst_morse_led_sim_create;
}

static void gst_morse_led_sim_init(GstMorseLEDSim *self) {
  self->message = g_strdup(DEFAULT_MESSAGE);
  self->wpm = DEFAULT_WPM;
  self->width = DEFAULT_WIDTH;
  self->height = DEFAULT_HEIGHT;
  self->framerate_num = DEFAULT_FRAMERATE_NUM;
  self->framerate_den = DEFAULT_FRAMERATE_DEN;
  self->dot_x = -1;
  self->dot_y = -1;
  self->dot_radius = DEFAULT_DOT_RADIUS;
  self->lead_in = DEFAULT_LEAD_IN;
  self->loop = DEFAULT_LOOP;
  self->frame_index = 0;
  self->frame_duration = 0.0;
  self->total_duration = 0.0;

  gst_video_info_init(&self->vinfo);
  gst_morse_led_sim_rebuild_timeline(self);

  gst_base_src_set_format(GST_BASE_SRC(self), GST_FORMAT_TIME);
  gst_base_src_set_live(GST_BASE_SRC(self), FALSE);
}

static void gst_morse_led_sim_finalize(GObject *object) {
  GstMorseLEDSim *self = GST_MORSE_LED_SIM(object);
  g_free(self->message);
  self->message = nullptr;
  G_OBJECT_CLASS(gst_morse_led_sim_parent_class)->finalize(object);
}

static void gst_morse_led_sim_set_property(GObject *object, guint prop_id,
                                           const GValue *value,
                                           GParamSpec *pspec) {
  GstMorseLEDSim *self = GST_MORSE_LED_SIM(object);

  switch (prop_id) {
  case PROP_MESSAGE:
    g_free(self->message);
    self->message = g_value_dup_string(value);
    gst_morse_led_sim_rebuild_timeline(self);
    break;
  case PROP_WPM:
    self->wpm = g_value_get_uint(value);
    gst_morse_led_sim_rebuild_timeline(self);
    break;
  case PROP_WIDTH:
    self->width = g_value_get_uint(value);
    break;
  case PROP_HEIGHT:
    self->height = g_value_get_uint(value);
    break;
  case PROP_FRAMERATE:
    self->framerate_num = gst_value_get_fraction_numerator(value);
    self->framerate_den = gst_value_get_fraction_denominator(value);
    break;
  case PROP_DOT_X:
    self->dot_x = g_value_get_int(value);
    break;
  case PROP_DOT_Y:
    self->dot_y = g_value_get_int(value);
    break;
  case PROP_DOT_RADIUS:
    self->dot_radius = g_value_get_uint(value);
    break;
  case PROP_LEAD_IN:
    self->lead_in = g_value_get_double(value);
    break;
  case PROP_LOOP:
    self->loop = g_value_get_boolean(value);
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    break;
  }
}

static void gst_morse_led_sim_get_property(GObject *object, guint prop_id,
                                           GValue *value, GParamSpec *pspec) {
  GstMorseLEDSim *self = GST_MORSE_LED_SIM(object);

  switch (prop_id) {
  case PROP_MESSAGE:
    g_value_set_string(value, self->message);
    break;
  case PROP_WPM:
    g_value_set_uint(value, self->wpm);
    break;
  case PROP_WIDTH:
    g_value_set_uint(value, self->width);
    break;
  case PROP_HEIGHT:
    g_value_set_uint(value, self->height);
    break;
  case PROP_FRAMERATE:
    gst_value_set_fraction(value, self->framerate_num, self->framerate_den);
    break;
  case PROP_DOT_X:
    g_value_set_int(value, self->dot_x);
    break;
  case PROP_DOT_Y:
    g_value_set_int(value, self->dot_y);
    break;
  case PROP_DOT_RADIUS:
    g_value_set_uint(value, self->dot_radius);
    break;
  case PROP_LEAD_IN:
    g_value_set_double(value, self->lead_in);
    break;
  case PROP_LOOP:
    g_value_set_boolean(value, self->loop);
    break;
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    break;
  }
}

static gboolean gst_morse_led_sim_start(GstBaseSrc *src) {
  GstMorseLEDSim *self = GST_MORSE_LED_SIM(src);

  if (self->framerate_num <= 0 || self->framerate_den <= 0) {
    GST_ERROR_OBJECT(self, "Invalid framerate %d/%d", self->framerate_num,
                     self->framerate_den);
    return FALSE;
  }

  gst_morse_led_sim_rebuild_timeline(self);
  self->frame_index = 0;
  self->frame_duration = static_cast<gdouble>(self->framerate_den) /
                         static_cast<gdouble>(self->framerate_num);

  gst_video_info_set_format(&self->vinfo, GST_VIDEO_FORMAT_RGB, self->width,
                            self->height);
  self->vinfo.fps_n = self->framerate_num;
  self->vinfo.fps_d = self->framerate_den;

  GstCaps *caps = gst_video_info_to_caps(&self->vinfo);
  gst_base_src_set_caps(src, caps);
  gst_caps_unref(caps);

  GST_INFO_OBJECT(self, "Starting Morse sim message=\"%s\" wpm=%u duration=%.3fs",
                  self->message, self->wpm, self->total_duration);
  return TRUE;
}

static gboolean gst_morse_led_sim_stop(GstBaseSrc *src) {
  GstMorseLEDSim *self = GST_MORSE_LED_SIM(src);
  self->frame_index = 0;
  return TRUE;
}

static GstFlowReturn gst_morse_led_sim_create(GstPushSrc *src,
                                              GstBuffer **buffer) {
  GstMorseLEDSim *self = GST_MORSE_LED_SIM(src);
  const gsize frame_size = GST_VIDEO_INFO_SIZE(&self->vinfo);

  GstBuffer *buf = gst_buffer_new_allocate(nullptr, frame_size, nullptr);
  if (buf == nullptr) {
    return GST_FLOW_ERROR;
  }

  gst_buffer_memset(buf, 0, 0, frame_size);

  const gdouble elapsed =
      static_cast<gdouble>(self->frame_index) * self->frame_duration;
  const gboolean led_on =
      elapsed >= self->lead_in &&
      timeline_led_on_at(self->timeline, elapsed - self->lead_in,
                         self->total_duration, self->loop);

  GstVideoFrame frame;
  if (!gst_video_frame_map(&frame, &self->vinfo, buf, GST_MAP_WRITE)) {
    gst_buffer_unref(buf);
    return GST_FLOW_ERROR;
  }

  gst_morse_led_sim_fill_black(&frame);
  if (led_on) {
    const gint cx =
        self->dot_x >= 0 ? self->dot_x : static_cast<gint>(self->width / 2);
    const gint cy =
        self->dot_y >= 0 ? self->dot_y : static_cast<gint>(self->height / 2);
    gst_morse_led_sim_draw_dot(&frame, cx, cy, self->dot_radius);
  }
  gst_video_frame_unmap(&frame);

  GST_BUFFER_PTS(buf) =
      static_cast<GstClockTime>(self->frame_index * self->frame_duration *
                                GST_SECOND);
  GST_BUFFER_DURATION(buf) =
      static_cast<GstClockTime>(self->frame_duration * GST_SECOND);

  *buffer = buf;
  self->frame_index++;

  if (!self->loop) {
    const gdouble next_elapsed =
        static_cast<gdouble>(self->frame_index) * self->frame_duration;
    if (next_elapsed > self->lead_in + self->total_duration) {
      return GST_FLOW_EOS;
    }
  }

  return GST_FLOW_OK;
}
