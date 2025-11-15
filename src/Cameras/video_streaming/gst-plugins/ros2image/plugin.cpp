#include "gstros2src.hpp"
#include <gst/gst.h>

static gboolean plugin_init(GstPlugin *plugin) {
  if (!gst_element_register(plugin, "ros2src", GST_RANK_NONE,
                            GST_TYPE_ROS2_IMAGE_SRC)) {
    return FALSE;
  }
  return TRUE;
}

GST_PLUGIN_DEFINE(
    GST_VERSION_MAJOR, GST_VERSION_MINOR, ros2, "ROS2 image source plugin",
    plugin_init, "1.0", "LGPL", "your-package-name",
    "https://github.com/CPRT/rover/tree/main/src/Cameras/video_streaming");
