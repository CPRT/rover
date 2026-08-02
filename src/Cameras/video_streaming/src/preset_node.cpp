#include "preset_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

PresetNode::PresetNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("preset_node", options) {
  declare_parameters();
  load_presets();

  auto presets_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

  presets_pub_ = this->create_publisher<interfaces::msg::VideoPresets>(
      "/video_presets", presets_qos);

  interfaces::msg::VideoPresets presets_msg;
  presets_msg.presets = presets_;
  presets_pub_->publish(presets_msg);

  RCLCPP_INFO(this->get_logger(), "PresetNode started");
}

void PresetNode::declare_parameters() {
  this->declare_parameter("presets", std::vector<std::string>());
  std::vector<std::string> preset_ids;
  this->get_parameter("presets", preset_ids);
  for (const auto &id : preset_ids) {
    this->declare_parameter(id + ".name", std::string());
    this->declare_parameter(id + ".sources", std::vector<std::string>());
    std::vector<std::string> sources;
    this->get_parameter(id + ".sources", sources);
    for (const auto &source : sources) {
      this->declare_parameter(id + "." + source + ".width", 0);
      this->declare_parameter(id + "." + source + ".height", 0);
      this->declare_parameter(id + "." + source + ".origin_x", 0);
      this->declare_parameter(id + "." + source + ".origin_y", 0);
    }
  }
}

void PresetNode::load_presets() {
  std::vector<std::string> preset_ids;
  this->get_parameter("presets", preset_ids);
  for (const auto &id : preset_ids) {
    interfaces::msg::VideoPreset preset;

    std::vector<std::string> sources;
    this->get_parameter(id + ".sources", sources);
    for (const auto &source_name : sources) {
      interfaces::msg::VideoSource source;
      source.name = source_name;
      source.width =
          this->get_parameter(id + "." + source_name + ".width").as_int();
      source.height =
          this->get_parameter(id + "." + source_name + ".height").as_int();
      source.origin_x =
          this->get_parameter(id + "." + source_name + ".origin_x").as_int();
      source.origin_y =
          this->get_parameter(id + "." + source_name + ".origin_y").as_int();
      preset.sources.push_back(source);
    }

    preset.name = this->get_parameter(id + ".name").as_string();
    presets_.push_back(preset);
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %ld presets", presets_.size());
}

RCLCPP_COMPONENTS_REGISTER_NODE(PresetNode)