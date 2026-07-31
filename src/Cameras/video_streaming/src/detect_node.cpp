#include "detect_node.hpp"
#include <cstdint>
#include <gstnvdsmeta.h>
#include <nvdsmeta.h>
#include <rclcpp_components/register_node_macro.hpp>

// Forward declaration of the detection callbacks (c style for gsignal)
static void on_marker_detected(GstElement *element, gint marker_id,
                               gpointer user_data);
static GstPadProbeReturn
metadata_probe_callback(GstPad *pad, GstPadProbeInfo *info, gpointer user_data);
static void on_morse_character_decoded(GstElement * /*element*/,
                                       guint char_code, gpointer user_data);
static void on_morse_calibrate_notify(GObject *element, GParamSpec * /*pspec*/,
                                      gpointer user_data);
static void on_morse_roi_locked(GstElement * /*element*/, gint roi_x,
                                gint roi_y, guint /*roi_width*/,
                                guint /*roi_height*/, gpointer user_data);

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
  declare_morse_parameters();
  param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&DetectNode::on_parameter_change, this, std::placeholders::_1));
  post_set_param_callback_handle_ = this->add_post_set_parameters_callback(
      std::bind(&DetectNode::on_parameters_set, this, std::placeholders::_1));
  marker_pub_ = this->create_publisher<std_msgs::msg::Int32>(
      "marker_detected", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  object_detected_pub_ =
      this->create_publisher<interfaces::msg::ObjectDetected>(
          "object_detected", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  morse_character_pub_ = this->create_publisher<std_msgs::msg::String>(
      "morse_character", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());
  morse_text_pub_ = this->create_publisher<std_msgs::msg::String>(
      "morse_text", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  start_pipeline();
}

void DetectNode::declare_morse_parameters() {
  this->declare_parameter<bool>("start_detection", false);
  this->declare_parameter<int>("roi_x", 0);
  this->declare_parameter<int>("roi_y", 0);
  this->declare_parameter<int>("roi_width", 96);
  this->declare_parameter<int>("roi_height", 96);
  this->declare_parameter<int>("wpm", 18);
  this->declare_parameter<bool>("calibrate", true);
  this->declare_parameter<double>("calibration_seconds", 2.0);
  this->declare_parameter<bool>("draw_roi", true);
  this->declare_parameter<double>("on_margin", 0.015);
  this->declare_parameter<double>("min_transition_units", 0.08);
  this->declare_parameter<double>("gap_detect_ratio", 0.8);
  this->declare_parameter<double>("dot_max_units", 1.8);
  this->declare_parameter<double>("dash_min_units", 2.2);
  this->declare_parameter<std::string>("metric_plot_path", "");
}

void DetectNode::apply_morse_element_properties(GstElement *morse) {
  g_object_set(
      morse, "start-detection",
      this->get_parameter("start_detection").as_bool() ? TRUE : FALSE, "roi-x",
      this->get_parameter("roi_x").as_int(), "roi-y",
      this->get_parameter("roi_y").as_int(), "roi-width",
      static_cast<guint>(this->get_parameter("roi_width").as_int()),
      "roi-height",
      static_cast<guint>(this->get_parameter("roi_height").as_int()), "wpm",
      static_cast<guint>(this->get_parameter("wpm").as_int()), "calibrate",
      this->get_parameter("calibrate").as_bool() ? TRUE : FALSE,
      "calibration-seconds",
      this->get_parameter("calibration_seconds").as_double(), "draw-roi",
      this->get_parameter("draw_roi").as_bool() ? TRUE : FALSE, "on-margin",
      static_cast<gfloat>(this->get_parameter("on_margin").as_double()),
      "min-transition-units",
      this->get_parameter("min_transition_units").as_double(),
      "gap-detect-ratio", this->get_parameter("gap_detect_ratio").as_double(),
      "dot-max-units", this->get_parameter("dot_max_units").as_double(),
      "dash-min-units", this->get_parameter("dash_min_units").as_double(),
      nullptr);

  const std::string plot_path =
      this->get_parameter("metric_plot_path").as_string();
  if (!plot_path.empty()) {
    g_object_set(morse, "metric-plot-path", plot_path.c_str(), nullptr);
  }
}

static std::string get_detection_pipeline_str(std::string config_path) {
  std::stringstream ss;
  ss << "nvvidconv ! queue ! mux.sink_0 "
     << "nvstreammux name=mux batch-size=1 width=1920 "
        "height=1080 live-source=1 ! queue ! nvinfer config-file-path="
     << config_path << " ! queue ! nvdsosd name=osd ! nvvidconv ! ";
  return ss.str();
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
    desc_ss << get_detection_pipeline_str(
        this->get_parameter("bottle_config").as_string());
    break;
  case DetectionType::MALLET:
    desc_ss << get_detection_pipeline_str(
        this->get_parameter("mallet_config").as_string());
    break;
  case DetectionType::ROCKPICK:
    desc_ss << get_detection_pipeline_str(
        this->get_parameter("rockpick_config").as_string());
    break;
  case DetectionType::ARUCO:
    desc_ss << "nvvidconv ! video/x-raw,format=NV12 ! videoconvert ! "
               "arucomarker name=aruco_detector detect-every=10 ! queue ! "
               "videoconvert ! video/x-raw,format=NV12 ! ";
    break;
  case DetectionType::MORSE:
    desc_ss << "nvvidconv ! video/x-raw,format=NV12 ! videoconvert ! "
               "morseLED name=morse_decoder ! queue ! videoconvert ! "
               "video/x-raw,format=NV12 ! ";
    break;
  case DetectionType::NONE:
    desc_ss << "identity ! ";
    break;
  }
  desc_ss << "nvvidconv ! video/x-raw(memory:NVMM),format=NV12 ! interpipesink "
             "name=detect";

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

  if (detection_type_ == DetectionType::MORSE) {
    GstElement *morse = gst_bin_get_by_name(GST_BIN(p), "morse_decoder");
    if (!morse) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get morseLED element.");
      gst_object_unref(p);
      return false;
    }
    apply_morse_element_properties(morse);
    gst_object_unref(morse);
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
  } else if (detection_type_ == DetectionType::MORSE) {
    GstElement *morse = get_element("morse_decoder");
    if (!morse) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get morseLED element.");
      return false;
    }
    g_signal_connect(morse, "character-decoded",
                     G_CALLBACK(on_morse_character_decoded), this);
    g_signal_connect(morse, "notify::calibrate",
                     G_CALLBACK(on_morse_calibrate_notify), this);
    g_signal_connect(morse, "roi-locked", G_CALLBACK(on_morse_roi_locked),
                     this);
    gst_object_unref(morse);
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
      obj_meta->rect_params.border_width = 6;

      guint class_id = obj_meta->class_id;
      gfloat confidence = obj_meta->confidence;
      NvOSD_RectParams bbox = obj_meta->rect_params;
      RCLCPP_INFO(self->get_logger(),
                  "Detected object with class_id=%d, confidence=%.2f, "
                  "bbox=[%d, %d, %d, %d]",
                  class_id, confidence, (int)bbox.left, (int)bbox.top,
                  (int)(bbox.left + bbox.width), (int)(bbox.top + bbox.height));

      self->publish_object_detected(
          static_cast<int32_t>(class_id), confidence,
          static_cast<int32_t>(bbox.left), static_cast<int32_t>(bbox.top),
          static_cast<int32_t>(bbox.left + bbox.width),
          static_cast<int32_t>(bbox.top + bbox.height));
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

void DetectNode::publish_morse_character(guint char_code) {
  std_msgs::msg::String msg;
  msg.data = std::string(1, static_cast<char>(char_code));
  RCLCPP_INFO(this->get_logger(), "Morse character decoded: '%s'",
              msg.data.c_str());
  if (morse_character_pub_) {
    morse_character_pub_->publish(msg);
  }

  GstElement *morse = get_element("morse_decoder");
  if (morse) {
    gchar *decoded = nullptr;
    g_object_get(morse, "decoded-text", &decoded, nullptr);
    if (decoded) {
      publish_morse_text(decoded);
      g_free(decoded);
    }
    gst_object_unref(morse);
  }
}

void DetectNode::publish_morse_text(const std::string &text) {
  std_msgs::msg::String msg;
  msg.data = text;
  if (morse_text_pub_) {
    morse_text_pub_->publish(msg);
  }
}

void DetectNode::sync_morse_calibrate_state(bool calibrating) {
  if (this->get_parameter("calibrate").as_bool() == calibrating) {
    return;
  }
  RCLCPP_INFO(this->get_logger(),
              "morseLED element reported calibrate=%s; syncing parameter.",
              calibrating ? "true" : "false");
  applying_morse_plugin_sync_ = true;
  this->set_parameters({rclcpp::Parameter("calibrate", calibrating)});
  applying_morse_plugin_sync_ = false;
}

void DetectNode::sync_morse_roi_state(int roi_x, int roi_y) {
  if (this->get_parameter("roi_x").as_int() == roi_x &&
      this->get_parameter("roi_y").as_int() == roi_y) {
    return;
  }
  RCLCPP_INFO(this->get_logger(),
              "morseLED element locked ROI to (%d, %d); syncing parameters.",
              roi_x, roi_y);
  applying_morse_plugin_sync_ = true;
  this->set_parameters(
      {rclcpp::Parameter("roi_x", roi_x), rclcpp::Parameter("roi_y", roi_y)});
  applying_morse_plugin_sync_ = false;
}

static void on_morse_character_decoded(GstElement * /*element*/,
                                       guint char_code, gpointer user_data) {
  auto *self = static_cast<DetectNode *>(user_data);
  if (!self) {
    return;
  }
  self->publish_morse_character(char_code);
}

static void on_morse_calibrate_notify(GObject *element, GParamSpec * /*pspec*/,
                                      gpointer user_data) {
  auto *self = static_cast<DetectNode *>(user_data);
  if (!self || !element) {
    return;
  }
  gboolean calibrating = FALSE;
  g_object_get(element, "calibrate", &calibrating, nullptr);
  self->sync_morse_calibrate_state(calibrating == TRUE);
}

static void on_morse_roi_locked(GstElement * /*element*/, gint roi_x,
                                gint roi_y, guint /*roi_width*/,
                                guint /*roi_height*/, gpointer user_data) {
  auto *self = static_cast<DetectNode *>(user_data);
  if (!self) {
    return;
  }
  self->sync_morse_roi_state(roi_x, roi_y);
}

void DetectNode::on_parameters_set(
    const std::vector<rclcpp::Parameter> &parameters) {
  if (applying_morse_plugin_sync_.load()) {
    return;
  }

  bool needs_restart = false;
  bool update_morse_element = false;
  for (const auto &param : parameters) {
    const std::string &name = param.get_name();
    if (name == "detection_type" || name == "listen_to") {
      needs_restart = true;
    } else if (name == "start_detection" || name == "roi_x" ||
               name == "roi_y" || name == "roi_width" || name == "roi_height" ||
               name == "wpm" || name == "calibrate" ||
               name == "calibration_seconds" || name == "draw_roi" ||
               name == "on_margin" || name == "min_transition_units" ||
               name == "gap_detect_ratio" || name == "dot_max_units" ||
               name == "dash_min_units" || name == "metric_plot_path") {
      update_morse_element = true;
    }
  }

  if (needs_restart) {
    RCLCPP_INFO(this->get_logger(),
                "Restarting pipeline to apply parameter changes.");
    stop_pipeline();
    if (!start_pipeline()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Failed to restart pipeline after parameter change.");
    }
    return;
  }

  if (update_morse_element && detection_type_ == DetectionType::MORSE) {
    GstElement *morse = get_element("morse_decoder");
    if (morse) {
      apply_morse_element_properties(morse);
      gst_object_unref(morse);
    }
  }
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
  } else if (type_str == "MORSE") {
    return DetectionType::MORSE;
  } else if (type_str == "NONE") {
    return DetectionType::NONE;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("DetectNode"),
                 "Invalid detection_type string: %s", type_str.c_str());
    return DetectionType::NONE;
  }
}

std::string DetectNode::detection_type_to_string() const {
  switch (detection_type_) {
  case DetectionType::WATER_BOTTLE:
    return "WATER_BOTTLE";
  case DetectionType::MALLET:
    return "MALLET";
  case DetectionType::ROCKPICK:
    return "ROCKPICK";
  case DetectionType::ARUCO:
    return "ARUCO";
  case DetectionType::MORSE:
    return "MORSE";
  case DetectionType::NONE:
  default:
    return "NONE";
  }
}

void DetectNode::publish_object_detected(int32_t class_id, float confidence,
                                         int32_t xmin, int32_t ymin,
                                         int32_t xmax, int32_t ymax) {
  auto msg = interfaces::msg::ObjectDetected();
  msg.model_type = detection_type_to_string();
  msg.class_id = class_id;
  msg.confidence = confidence;
  msg.xmin = xmin;
  msg.ymin = ymin;
  msg.xmax = xmax;
  msg.ymax = ymax;
  if (object_detected_pub_) {
    object_detected_pub_->publish(msg);
  }
}

rcl_interfaces::msg::SetParametersResult DetectNode::on_parameter_change(
    const std::vector<rclcpp::Parameter> &parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  for (const auto &param : parameters) {
    if (param.get_name() == "bottle_config" ||
        param.get_name() == "mallet_config" ||
        param.get_name() == "rockpick_config") {
      RCLCPP_WARN(this->get_logger(),
                  "Changing model configs can only be done at startup.");
    } else if (param.get_name() == "detection_type") {
      const std::string type_str = param.as_string();
      if (type_str != "WATER_BOTTLE" && type_str != "MALLET" &&
          type_str != "ROCKPICK" && type_str != "ARUCO" &&
          type_str != "MORSE" && type_str != "NONE") {
        result.successful = false;
        result.reason = "Invalid detection_type: " + type_str;
        return result;
      }
      RCLCPP_INFO(this->get_logger(), "detection_type set to %s",
                  type_str.c_str());
    } else if (param.get_name() == "listen_to") {
      RCLCPP_INFO(this->get_logger(), "Changing listen_to parameter to %s.",
                  param.as_string().c_str());
    }
  }
  return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(DetectNode)
