#pragma once

#include <gst/gst.h>
#include <gst/video/gstvideofilter.h>

G_BEGIN_DECLS

#define GST_TYPE_MORSE_LED (gst_morse_led_get_type())
G_DECLARE_FINAL_TYPE(GstMorseLED, gst_morse_led, GST, MORSE_LED, GstVideoFilter)

G_END_DECLS
