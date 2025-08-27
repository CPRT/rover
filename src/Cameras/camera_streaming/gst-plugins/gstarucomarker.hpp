#pragma once

#include <gst/gst.h>
#include <gst/video/gstvideofilter.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

G_BEGIN_DECLS

#define GST_TYPE_ARUCOMARKER (gst_arucomarker_get_type())
G_DECLARE_FINAL_TYPE(GstArucoMarker, gst_arucomarker, GST, ARUCOMARKER,
                     GstVideoFilter)

G_END_DECLS
