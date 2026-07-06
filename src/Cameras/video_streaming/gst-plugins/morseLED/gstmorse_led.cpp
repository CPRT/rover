#include "gstmorse_led.hpp"

#include "morse_decoder.hpp"

GST_DEBUG_CATEGORY_STATIC(morse_led_debug);
#define GST_CAT_DEFAULT morse_led_debug

enum {
  PROP_0,
  /* TODO: add properties, e.g. PROP_START_DETECTION, PROP_DRAW_OVERLAY,
   * PROP_WPM, PROP_DECODED_TEXT, PROP_ROI_X, ... */
};

/* Pad templates ---------------------------------------------------------------- */

static GstStaticPadTemplate sink_template = GST_STATIC_PAD_TEMPLATE(
    "sink", GST_PAD_SINK, GST_PAD_ALWAYS,
    GST_STATIC_CAPS("video/x-raw, format=(string)RGB, "
                    "framerate=(fraction)[0/1, 2147483647/1]"));

static GstStaticPadTemplate src_template = GST_STATIC_PAD_TEMPLATE(
    "src", GST_PAD_SRC, GST_PAD_ALWAYS,
    GST_STATIC_CAPS("video/x-raw, format=(string)RGB, "
                    "framerate=(fraction)[0/1, 2147483647/1]"));

/* Instance data ---------------------------------------------------------------- */

struct _GstMorseLED {
  GstVideoFilter parent;

  /* TODO: GObject-owned fields (properties, strings, flags) */
  /* TODO: MorseDecoder *decoder; or embed MorseDecoder directly */
};

/* Forward declarations --------------------------------------------------------- */

static void gst_morse_led_init(GstMorseLED *self);
static void gst_morse_led_class_init(GstMorseLEDClass *klass);

static void gst_morse_led_set_property(GObject *object, guint prop_id,
                                       const GValue *value, GParamSpec *pspec);
static void gst_morse_led_get_property(GObject *object, guint prop_id,
                                       GValue *value, GParamSpec *pspec);

static GstFlowReturn gst_morse_led_transform_frame_ip(GstVideoFilter *filter,
                                                      GstVideoFrame *frame);

G_DEFINE_TYPE(GstMorseLED, gst_morse_led, GST_TYPE_VIDEO_FILTER)

/* Instance init ---------------------------------------------------------------- */

static void gst_morse_led_init(GstMorseLED *self) {
  /* TODO: set default property values and allocate decoder */
  (void)self;
}

/* Property accessors ----------------------------------------------------------- */

static void gst_morse_led_set_property(GObject *object, guint prop_id,
                                       const GValue *value, GParamSpec *pspec) {
  GstMorseLED *self = GST_MORSE_LED(object);

  switch (prop_id) {
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    break;
  }

  (void)self;
  (void)value;
}

static void gst_morse_led_get_property(GObject *object, guint prop_id,
                                       GValue *value, GParamSpec *pspec) {
  GstMorseLED *self = GST_MORSE_LED(object);

  switch (prop_id) {
  default:
    G_OBJECT_WARN_INVALID_PROPERTY_ID(object, prop_id, pspec);
    break;
  }

  (void)self;
  (void)value;
}

/* Class init ------------------------------------------------------------------- */

static void gst_morse_led_class_init(GstMorseLEDClass *klass) {
  GST_DEBUG_CATEGORY_INIT(morse_led_debug, "morseLED", 0,
                          "Morse LED decoder");

  GObjectClass *gobject_class = G_OBJECT_CLASS(klass);
  GstElementClass *element_class = GST_ELEMENT_CLASS(klass);
  GstVideoFilterClass *vfilter_class = GST_VIDEO_FILTER_CLASS(klass);

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

  vfilter_class->transform_frame_ip = gst_morse_led_transform_frame_ip;

  /* TODO: install properties with g_object_class_install_property() */

  /* TODO: register signals, e.g.:
   *   "roi-locked"         (x, y, width, height)
   *   "symbol-detected"    (dit/dah)
   *   "character-decoded"  (gchar)
   *   "lock-failed"        ()
   */
}

/* Frame processing ------------------------------------------------------------- */

static GstFlowReturn gst_morse_led_transform_frame_ip(GstVideoFilter *filter,
                                                      GstVideoFrame *frame) {
  GstMorseLED *self = GST_MORSE_LED(filter);

  /* TODO:
   * 1. Read buffer PTS → timestamp in seconds
   * 2. Compute brightness metric inside ROI (from GstVideoFrame plane data)
   * 3. morse_decoder_process_sample(...)
   * 4. Emit signals when decoder reports events
   * 5. Optionally draw debug overlay when enabled
   */

  (void)self;
  (void)frame;

  return GST_FLOW_OK;
}
