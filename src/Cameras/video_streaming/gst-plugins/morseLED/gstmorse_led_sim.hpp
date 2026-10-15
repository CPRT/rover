#pragma once

#include <gst/base/gstpushsrc.h>
#include <gst/gst.h>
#include <gst/video/video.h>

G_BEGIN_DECLS

#define GST_TYPE_MORSE_LED_SIM (gst_morse_led_sim_get_type())
G_DECLARE_FINAL_TYPE(GstMorseLEDSim, gst_morse_led_sim, GST, MORSE_LED_SIM,
                     GstPushSrc)

G_END_DECLS
