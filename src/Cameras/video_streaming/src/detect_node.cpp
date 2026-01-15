#include "detect_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

// Forward declaration of the marker detected callback (c style for gsignal)
static void on_marker_detected(GstElement *element, gint marker_id,
                               gpointer user_data);

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

  switch (detection_type_) {
  case DetectionType::WATER_BOTTLE:
    desc_ss << "nvvidconv ! queue ! mux.sink_0 nvstreammux name=mux "
               "batch-size=1 width=1920 height=1080 live-source=1 ! queue ! "
               "nvinfer config-file-path="
            << this->get_parameter("bottle_config").as_string()
            << " ! queue ! nvdsosd ! nvvidconv ! ";
    break;
  case DetectionType::MALLET:
    RCLCPP_ERROR(this->get_logger(), "Mallet detection not implemented yet.");
    desc_ss << "identity ! ";
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
  }
  return true;
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
      std::string type_str = param.as_string();
      if (type_str == "WATER_BOTTLE") {
        detection_type_ = DetectionType::WATER_BOTTLE;
      } else if (type_str == "MALLET") {
        detection_type_ = DetectionType::MALLET;
      } else if (type_str == "ROCKPICK") {
        detection_type_ = DetectionType::ROCKPICK;
      } else if (type_str == "ARUCO") {
        detection_type_ = DetectionType::ARUCO;
      } else if (type_str == "NONE") {
        detection_type_ = DetectionType::NONE;
      } else {
        result.successful = false;
        result.reason = "Invalid detection_type: " + type_str;
        continue;
      }
      RCLCPP_INFO(this->get_logger(), "detection_type set to %s",
                  type_str.c_str());
      if (!stop_pipeline()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to stop pipeline for reconfiguration.");
        result.successful = false;
        result.reason = "Failed to stop pipeline for reconfiguration.";
        continue;
      }
      if (!start_pipeline()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to start pipeline after reconfiguration.");
        result.successful = false;
        result.reason = "Failed to start pipeline after reconfiguration.";
        continue;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Pipeline restarted with new detection_type %s.",
                  type_str.c_str());
    } else if (param.get_name() == "listen_to") {
      if (!stop_pipeline()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to stop pipeline for reconfiguration.");
        result.successful = false;
        result.reason = "Failed to stop pipeline for reconfiguration.";
        continue;
      }
      if (!start_pipeline()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to start pipeline after reconfiguration.");
        result.successful = false;
        result.reason = "Failed to start pipeline after reconfiguration.";
        continue;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Pipeline restarted with new listen_to parameter.");
    }
  }
  return result;
}

RCLCPP_COMPONENTS_REGISTER_NODE(DetectNode)
