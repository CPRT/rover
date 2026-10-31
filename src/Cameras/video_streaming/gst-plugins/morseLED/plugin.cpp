#include "gstmorse_led.hpp"
#include "gstmorse_led_sim.hpp"
#include <gst/gst.h>

static gboolean plugin_init(GstPlugin *plugin) {
  if (!gst_element_register(plugin, "morseLED", GST_RANK_NONE,
                            GST_TYPE_MORSE_LED)) {
    return FALSE;
  }
  if (!gst_element_register(plugin, "morseLEDSim", GST_RANK_NONE,
                            GST_TYPE_MORSE_LED_SIM)) {
    return FALSE;
  }
  return TRUE;
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR, GST_VERSION_MINOR, morseLED,
    "Morse LED decoder and test source", plugin_init, "1.0", "LGPL", "morseLED",
    "https://github.com/CPRT/rover/tree/main/src/Cameras/video_streaming")
