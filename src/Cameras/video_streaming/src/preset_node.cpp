#include "preset_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>

PresetNode::PresetNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("preset_node", options) {
  declare_parameters();
  load_presets();

  preset_service_ = this->create_service<interfaces::srv::VideoPreset>(
      "/request_preset",
      std::bind(&PresetNode::preset_cb, this, std::placeholders::_1,
                std::placeholders::_2));

  list_presets_service_ = this->create_service<interfaces::srv::GetPresets>(
      "/list_presets",
      [this](
          const std::shared_ptr<interfaces::srv::GetPresets::Request> request,
          std::shared_ptr<interfaces::srv::GetPresets::Response> response) {
        for (auto const &preset : presets_) {
          response->presets.push_back(preset.first);
        }
      });

  video_client_ =
      this->create_client<interfaces::srv::VideoOut>("/start_video");

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
    interfaces::srv::VideoOut::Request preset;

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

    std::string name = this->get_parameter(id + ".name").as_string();
    presets_.insert({name, preset});
  }

  RCLCPP_INFO(this->get_logger(), "Loaded %ld presets", presets_.size());
}

void PresetNode::preset_cb(
    const std::shared_ptr<interfaces::srv::VideoPreset::Request> request,
    std::shared_ptr<interfaces::srv::VideoPreset::Response> response) {
  if (auto search = presets_.find(request->name); search != presets_.end()) {
    if (!video_client_->wait_for_service(std::chrono::seconds(2))) {
      RCLCPP_ERROR(this->get_logger(),
                   "Start video service is unavailable!!!!");
      response->success = false;
      return;
    }

    auto video_request =
        std::make_shared<interfaces::srv::VideoOut::Request>(search->second);

    auto result = video_client_->async_send_request(video_request);

    if (result.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Timed out selecting preset %s!!!!",
                   request->name.c_str());
      response->success = false;
      return;
    }

    const auto video_response = result.get();
    if (!video_response->success) {
      RCLCPP_ERROR(this->get_logger(), "Failed to select preset %s!!!!",
                   request->name.c_str());
    }
    response->success = video_response->success;
  } else {
    RCLCPP_WARN(this->get_logger(), "Could not find preset: %s",
                request->name.c_str());
    response->success = false;
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(PresetNode)