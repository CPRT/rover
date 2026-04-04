#include "detect_node.hpp"
#include <gstnvdsmeta.h>
#include <nvdsmeta.h>
#include <rclcpp_components/register_node_macro.hpp>

// Forward declaration of the detection callbacks (c style for gsignal)
static void on_marker_detected(GstElement *element, gint marker_id,
                               gpointer user_data);
static GstPadProbeReturn
metadata_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);

DetectNode::DetectNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("detect_node", options),
      detection_type_(DetectionType::NONE) {
  this->declare_parameter<std::string>("bottle_config",
                                       "config/bottle/bottle.txt");
  this->declare_parameter<std::string>("detection_type", "NONE");
  this->declare_parameter<std::string>("mallet_config",
                                       "config/mallet/mallet.txt");
  this->declare_parameter<std::string>("rockpick_config",
                                       "config/rockpick/rockpick.txt");
  this->declare_parameter<std::string>("listen_to", "input");
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DetectNode::on_parameter_change, this, std::placeholders::_1));
  marker_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "marker_detected", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  start_pipeline();
}

bool DetectNode::create_pipeline() {

  std::stringstream desc_ss;
  desc_ss << "interpipesrc format=3 listen-to="
          << this->get_parameter("listen_to").as_string()
          << " is-live=true "
             "allow-renegotiation=true name=src ! ";
  detection_type_ = string_to_detection_type(
      this->get_parameter("detection_type").as_string());
  switch (detection_type_) {
  case DetectionType::WATER_BOTTLE:
    desc_ss << "nvvidconv ! queue ! mux.sink_0 nvstreammux name=mux "
               "batch-size=1 width=1920 height=1080 live-source=1 ! queue ! "
               "nvinfer config-file-path="
            << this->get_parameter("bottle_config").as_string()
            << " ! queue ! nvdsosd ! nvvidconv ! ";
    break;
  case DetectionType::MALLET:
    desc_ss << "nvvidconv ! queue ! mux.sink_0 nvstreammux name=mux "
               "batch-size=1 width=1920 height=1080 live-source=1 ! queue ! "
               "nvinfer config-file-path="
            << this->get_parameter("mallet_config").as_string()
            << " ! queue ! nvdsosd name=osd ! nvvidconv ! ";
    break;
  case DetectionType::ROCKPICK:
    RCLCPP_ERROR(this->get_logger(), "Rockpick detection not implemented yet.");
    desc_ss << "identity ! ";
    break;
  case DetectionType::ARUCO:
    desc_ss << "videoconvert ! queue ! videoconvert ! arucomarker "
               "detect-every=10 "
               "name=aruco_detector ! queue ! videoconvert ! ";
    break;
  case DetectionType::NONE:
    desc_ss << "identity ! ";
    break;
  }
  desc_ss << "nvvidconv ! interpipesink name=detect";

  RCLCPP_INFO(this->get_logger(), "Creating pipeline: %s",
              desc_ss.str().c_str());

  GError *err = nullptr;
  GstElement *p = gst_parse_launch(desc_ss.str().c_str(), &err);
  if (err || !p) {
    RCLCPP_ERROR(this->get_logger(), "gst_parse_launch failed: %s",
                 err ? err->message : "unknown");
    if (err)
      g_error_free(err);
    return false;
  }
  if (!GST_IS_PIPELINE(p)) {
    RCLCPP_ERROR(this->get_logger(), "Parsed element is not a pipeline.");
    gst_object_unref(p);
    return false;
  }
  if (pipeline_) {
    RCLCPP_ERROR(this->get_logger(),
                 "Pipeline already exists in create_pipeline.");
    gst_object_unref(p);
    return false;
  }

  pipeline_ = p;
  return true;
}
bool DetectNode::start_pipeline() {
  if (!BaseVideoNode::start_pipeline()) {
    return false;
  }
  if (detection_type_ == DetectionType::ARUCO) {
    GstElement *aruco = get_element("aruco_detector");
    if (!aruco) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get arucomarker element.");
      return false;
    }
    g_signal_connect(aruco, "marker-detected", G_CALLBACK(on_marker_detected),
                     marker_pub_.get());
    gst_object_unref(aruco);
  } else if (detection_type_ == DetectionType::MALLET ||
             detection_type_ == DetectionType::WATER_BOTTLE) {
    GstElement *osd = get_element("osd");
    if (!osd) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get OSD element.");
      return false;
    }
    gst_pad_add_probe(gst_element_get_static_pad(osd, "sink"),
                      GST_PAD_PROBE_TYPE_BUFFER, metadata_probe_callback, this,
                      nullptr);
    gst_object_unref(osd);
  }
  return true;
}

GstPadProbeReturn metadata_probe_callback(GstPad *pad, GstPadProbeInfo *info,
                                          gpointer user_data) {
  // TODO: (ERIK) Create whatever API you want for autonomy
  DetectNode *self = static_cast<DetectNode *>(user_data);
  GstBuffer *buffer = GST_PAD_PROBE_INFO_BUFFER(info);
  if (!buffer) {
    return GST_PAD_PROBE_PASS;
  }
  NvDsBatchMeta *batch_meta = gst_buffer_get_nvds_batch_meta(buffer);
  if (!batch_meta) {
    return GST_PAD_PROBE_PASS;
  }
  NvDsMetaList *l_frame = NULL;

  for (l_frame = batch_meta->frame_meta_list; l_frame != NULL;
       l_frame = l_frame->next) {
    NvDsFrameMeta *frame_meta = (NvDsFrameMeta *)(l_frame->data);
    NvDsMetaList *l_obj = NULL;
    for (l_obj = frame_meta->obj_meta_list; l_obj != NULL;
         l_obj = l_obj->next) {
      NvDsObjectMeta *obj_meta = (NvDsObjectMeta *)(l_obj->data);
      guint class_id = obj_meta->class_id;
      gfloat confidence = obj_meta->confidence;
      NvOSD_RectParams bbox = obj_meta->rect_params;
      RCLCPP_INFO(self->get_logger(),
                  "Detected object with class_id=%d, confidence=%.2f, "
                  "bbox=[%d, %d, %d, %d]",
                  class_id, confidence, (int)bbox.left, (int)bbox.top,
                  (int)(bbox.left + bbox.width), (int)(bbox.top + bbox.height));
    }
  }
  return GST_PAD_PROBE_PASS;
}

static void on_marker_detected(GstElement *element, gint marker_id,
                               gpointer user_data) {
  auto *marker_pub =
      static_cast<rclcpp::Publisher<std_msgs::msg::Int32> *>(user_data);
  std_msgs::msg::Int32 msg;
  msg.data = marker_id;
  RCLCPP_INFO(rclcpp::get_logger("Aruco_Detector"),
              "Marker detected with ID: %d", marker_id);
  if (!marker_pub) {
    RCLCPP_ERROR(
        rclcpp::get_logger("Aruco_Detector"),
        "Marker publisher is not initialized, cannot publish marker ID");
    return;
  }
  marker_pub->publish(msg);
}

DetectNode::DetectionType
DetectNode::string_to_detection_type(const std::string &type_str) {
  if (type_str == "WATER_BOTTLE") {
    return DetectionType::WATER_BOTTLE;
  } else if (type_str == "MALLET") {
    return DetectionType::MALLET;
  } else if (type_str == "ROCKPICK") {
    return DetectionType::ROCKPICK;
  } else if (type_str == "ARUCO") {
    return DetectionType::ARUCO;
  } else if (type_str == "NONE") {
    return DetectionType::NONE;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("DetectNode"),
                 "Invalid detection_type string: %s", type_str.c_str());
    return DetectionType::NONE;
  }
}

rcl_interfaces::msg::SetParametersResult DetectNode::on_parameter_change(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  bool needs_restart = false;
  for (const auto &param : parameters) {
    if (param.get_name() == "bottle_config" ||
        param.get_name() == "mallet_config" ||
        param.get_name() == "rockpick_config") {
      RCLCPP_WARN(this->get_logger(),
                  "Changing model configs can only be done at startup.");
    } else if (param.get_name() == "detection_type") {
      std::string type_str = param.as_string();
      detection_type_ = string_to_detection_type(type_str);
      RCLCPP_INFO(this->get_logger(), "detection_type set to %s",
                  type_str.c_str());
      needs_restart = true;
    } else if (param.get_name() == "listen_to") {
      RCLCPP_INFO(this->get_logger(), "Changing listen_to parameter to %s.",
                  param.as_string().c_str());
      needs_restart = true;
    }
  }
  if (needs_restart) {
    RCLCPP_INFO(this->get_logger(),
                "Restarting pipeline to apply parameter changes.");
    stop_pipeline();
    if (!start_pipeline()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to restart pipeline after parameter change.");
      result.successful = false;
      result.reason = "Failed to restart pipeline after parameter change.";
    }
  }
  return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(DetectNode)
