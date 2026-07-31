#ifndef PRESET_NODE_HPP
#define PRESET_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/video_preset.hpp"
#include "interfaces/srv/get_presets.hpp"

class PresetNode : public rclcpp::Node {
public:
  explicit PresetNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void load_presets();

  std::vector<interfaces::msg::VideoPreset> presets_;
  rclcpp::Service<interfaces::srv::GetPresets>::SharedPtr list_presets_service_;
};

#endif // PRESET_NODE_HPP