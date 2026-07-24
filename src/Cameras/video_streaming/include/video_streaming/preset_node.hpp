#ifndef PRESET_NODE_HPP
#define PRESET_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include "interfaces/srv/get_presets.hpp"
#include "interfaces/srv/video_out.hpp"
#include "interfaces/srv/video_preset.hpp"

class PresetNode : public rclcpp::Node {
public:
  explicit PresetNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  void declare_parameters();
  void load_presets();

  void preset_cb(
      const std::shared_ptr<interfaces::srv::VideoPreset::Request> request,
      std::shared_ptr<interfaces::srv::VideoPreset::Response> response);

  std::map<std::string, interfaces::srv::VideoOut::Request> presets_;
  rclcpp::Service<interfaces::srv::VideoPreset>::SharedPtr preset_service_;
  rclcpp::Service<interfaces::srv::GetPresets>::SharedPtr list_presets_service_;
  rclcpp::Client<interfaces::srv::VideoOut>::SharedPtr video_client_;
};

#endif // PRESET_NODE_HPP