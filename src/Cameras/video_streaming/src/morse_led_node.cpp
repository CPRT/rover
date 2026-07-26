#include "morse_led_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <sstream>

static void on_character_decoded(GstElement * /*element*/, guint char_code,
                                 gpointer user_data);
static void on_calibrate_notify(GObject *element, GParamSpec * /*pspec*/,
                                gpointer user_data);
static void on_roi_locked(GstElement * /*element*/, gint roi_x, gint roi_y,
                          guint /*roi_width*/, guint /*roi_height*/,
                          gpointer user_data);

MorseLedNode::MorseLedNode(const rclcpp::NodeOptions &options)
    : BaseVideoNode("morse_led_node", options) {
  declare_parameters();
  post_set_param_callback_handle_ = this->add_post_set_parameters_callback(
      std::bind(&MorseLedNode::on_parameters_set, this, std::placeholders::_1));
  character_pub_ = this->create_publisher<std_msgs::msg::String>(
      "morse_character", rclcpp::QoS(rclcpp::KeepLast(50)).reliable());
  text_pub_ = this->create_publisher<std_msgs::msg::String>(
      "morse_text", rclcpp::QoS(rclcpp::KeepLast(10)).reliable());
  start_pipeline();
}

void MorseLedNode::declare_parameters() {
  this->declare_parameter<std::string>("listen_to", "input");
  this->declare_parameter<bool>("start_detection", true);
  this->declare_parameter<int>("roi_x", 0);
  this->declare_parameter<int>("roi_y", 0);
  this->declare_parameter<int>("roi_width", 96);
  this->declare_parameter<int>("roi_height", 96);
  this->declare_parameter<int>("wpm", 18);
  this->declare_parameter<bool>("calibrate", false);
  this->declare_parameter<double>("calibration_seconds", 2.0);
  this->declare_parameter<bool>("draw_roi", false);
  this->declare_parameter<double>("on_margin", 0.015);
  this->declare_parameter<double>("min_transition_units", 0.08);
  this->declare_parameter<double>("gap_detect_ratio", 0.8);
  this->declare_parameter<double>("dot_max_units", 1.8);
  this->declare_parameter<double>("dash_min_units", 2.2);
  this->declare_parameter<std::string>("metric_plot_path", "");
}

void MorseLedNode::apply_element_properties(GstElement *morse) {
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

bool MorseLedNode::create_pipeline() {
  std::stringstream desc_ss;
  desc_ss << "interpipesrc format=3 listen-to="
          << this->get_parameter("listen_to").as_string()
          << " is-live=true "
             "allow-renegotiation=true name=src ! "
             "videoconvert ! queue ! videoconvert ! "
             "morseLED name=morse_decoder ! queue ! videoconvert ! "
             "nvvidconv ! interpipesink name=morse";

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

  GstElement *morse = gst_bin_get_by_name(GST_BIN(p), "morse_decoder");
  if (!morse) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get morseLED element.");
    gst_object_unref(p);
    return false;
  }
  apply_element_properties(morse);
  gst_object_unref(morse);

  pipeline_ = p;
  return true;
}

bool MorseLedNode::start_pipeline() {
  if (!BaseVideoNode::start_pipeline()) {
    return false;
  }

  GstElement *morse = get_element("morse_decoder");
  if (!morse) {
    RCLCPP_ERROR(this->get_logger(), "Failed to get morseLED element.");
    return false;
  }
  g_signal_connect(morse, "character-decoded", G_CALLBACK(on_character_decoded),
                   this);
  g_signal_connect(morse, "notify::calibrate", G_CALLBACK(on_calibrate_notify),
                   this);
  g_signal_connect(morse, "roi-locked", G_CALLBACK(on_roi_locked), this);
  gst_object_unref(morse);
  return true;
}

void MorseLedNode::publish_character(guint char_code) {
  std_msgs::msg::String msg;
  msg.data = std::string(1, static_cast<char>(char_code));
  RCLCPP_INFO(this->get_logger(), "Morse character decoded: '%s'",
              msg.data.c_str());
  if (character_pub_) {
    character_pub_->publish(msg);
  }

  GstElement *morse = get_element("morse_decoder");
  if (morse) {
    gchar *decoded = nullptr;
    g_object_get(morse, "decoded-text", &decoded, nullptr);
    if (decoded) {
      publish_decoded_text(decoded);
      g_free(decoded);
    }
    gst_object_unref(morse);
  }
}

void MorseLedNode::publish_decoded_text(const std::string &text) {
  std_msgs::msg::String msg;
  msg.data = text;
  if (text_pub_) {
    text_pub_->publish(msg);
  }
}

void MorseLedNode::sync_calibrate_state(bool calibrating) {
  if (this->get_parameter("calibrate").as_bool() == calibrating) {
    return;
  }
  RCLCPP_INFO(this->get_logger(),
              "morseLED element reported calibrate=%s; syncing parameter.",
              calibrating ? "true" : "false");
  applying_plugin_sync_ = true;
  this->set_parameters({rclcpp::Parameter("calibrate", calibrating)});
  applying_plugin_sync_ = false;
}

void MorseLedNode::sync_roi_state(int roi_x, int roi_y) {
  if (this->get_parameter("roi_x").as_int() == roi_x &&
      this->get_parameter("roi_y").as_int() == roi_y) {
    return;
  }
  RCLCPP_INFO(this->get_logger(),
              "morseLED element locked ROI to (%d, %d); syncing parameters.",
              roi_x, roi_y);
  applying_plugin_sync_ = true;
  this->set_parameters(
      {rclcpp::Parameter("roi_x", roi_x), rclcpp::Parameter("roi_y", roi_y)});
  applying_plugin_sync_ = false;
}

static void on_character_decoded(GstElement * /*element*/, guint char_code,
                                 gpointer user_data) {
  auto *self = static_cast<MorseLedNode *>(user_data);
  if (!self) {
    return;
  }
  self->publish_character(char_code);
}

static void on_calibrate_notify(GObject *element, GParamSpec * /*pspec*/,
                                gpointer user_data) {
  auto *self = static_cast<MorseLedNode *>(user_data);
  if (!self || !element) {
    return;
  }
  gboolean calibrating = FALSE;
  g_object_get(element, "calibrate", &calibrating, nullptr);
  self->sync_calibrate_state(calibrating == TRUE);
}

static void on_roi_locked(GstElement * /*element*/, gint roi_x, gint roi_y,
                          guint /*roi_width*/, guint /*roi_height*/,
                          gpointer user_data) {
  auto *self = static_cast<MorseLedNode *>(user_data);
  if (!self) {
    return;
  }
  self->sync_roi_state(roi_x, roi_y);
}

void MorseLedNode::on_parameters_set(
    const std::vector<rclcpp::Parameter> &parameters) {
  if (applying_plugin_sync_.load()) {
    return;
  }

  bool needs_restart = false;
  bool update_element = false;

  for (const auto &param : parameters) {
    const std::string &name = param.get_name();
    if (name == "listen_to") {
      RCLCPP_INFO(this->get_logger(), "Changing listen_to parameter to %s.",
                  param.as_string().c_str());
      needs_restart = true;
    } else if (name == "start_detection" || name == "roi_x" ||
               name == "roi_y" || name == "roi_width" || name == "roi_height" ||
               name == "wpm" || name == "calibrate" ||
               name == "calibration_seconds" || name == "draw_roi" ||
               name == "on_margin" || name == "min_transition_units" ||
               name == "gap_detect_ratio" || name == "dot_max_units" ||
               name == "dash_min_units" || name == "metric_plot_path") {
      update_element = true;
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

  if (update_element) {
    GstElement *morse = get_element("morse_decoder");
    if (morse) {
      apply_element_properties(morse);
      gst_object_unref(morse);
    }
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(MorseLedNode)
