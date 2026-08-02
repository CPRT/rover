#ifndef PRESET_NODE_HPP
#define PRESET_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/video_preset.hpp"
#include "interfaces/msg/video_presets.hpp"

class PresetNode : public rclcpp::Node {
public:
  explicit PresetNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void load_presets();

  std::vector<interfaces::msg::VideoPreset> presets_;
  rclcpp::Publisher<interfaces::msg::VideoPresets>::SharedPtr presets_pub_;
};

#endif // PRESET_NODE_HPP