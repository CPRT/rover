#include "../include/video_streaming/camera_settings_server.hpp"

#include <array>
#include <cstdio>
#include <fstream>
#include <regex>
#include <sstream>
#include <vector>

namespace video_streaming {

CameraSettingsServer::CameraSettingsServer(const rclcpp::NodeOptions &options)
    : Node("camera_settings_server", options) {

  declare_parameter<std::string>("camera_config", "camera_config.yaml");

  config_file_ = get_parameter("camera_config").as_string();
  std::string default_config =
      "./setup/v4l2_settings_controller/camera_config.yaml";

  try {

    load_config(config_file_);

  } catch (const std::exception &e) {

    RCLCPP_WARN(get_logger(), "Failed loading %s", config_file_.c_str());

    try {
      config_file_ = default_config;
      load_config(default_config);
    } catch (const std::exception &e) {
      RCLCPP_WARN(get_logger(), "Failed loading %s", default_config.c_str());
    }
  }

  service_ = create_service<GetCamera>(
      "get_camera", std::bind(&CameraSettingsServer::handle_request, this,
                              std::placeholders::_1, std::placeholders::_2));

  yaml_service_ = create_service<GetCamera>(
      "get_camera_yaml",
      std::bind(&CameraSettingsServer::get_from_yaml, this,
                std::placeholders::_1, std::placeholders::_2));

  set_service_ = create_service<SetCamera>(
      "set_camera", std::bind(&CameraSettingsServer::set_camera, this,
                              std::placeholders::_1, std::placeholders::_2));

  save_service_ = create_service<SetCamera>(
      "save_camera", std::bind(&CameraSettingsServer::save_camera, this,
                               std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(get_logger(), "Camera settings services ready");
}

// --------------------------------------------------
// YAML LOAD
// --------------------------------------------------

void CameraSettingsServer::load_config(const std::string &filename) {

  YAML::Node config = YAML::LoadFile(filename);

  for (auto camera : config) {

    std::string name = camera.first.as<std::string>();

    CameraInfo info;

    info.path = camera.second["path"].as<std::string>();

    for (auto ctrl : camera.second["controls"]) {

      std::string ctrl_name = ctrl.first.as<std::string>();

      info.controls[ctrl_name] = ctrl.second;
    }

    cameras_[name] = info;

    RCLCPP_INFO(get_logger(), "Loaded camera %s", name.c_str());
  }
}

// --------------------------------------------------
// YAML SAVE
// --------------------------------------------------

bool CameraSettingsServer::save_config(const std::string &camera_name) {
  auto camera = cameras_.find(camera_name);

  if (camera == cameras_.end())
    return false;

  YAML::Node output;

  for (auto &[name, value] : camera->second.current_values) {
    output[name]["value"] = value;
  }

  std::filesystem::path config_path(config_file_);

  std::string filename =
      (config_path.parent_path() / (camera_name + "_settings.yaml")).string();

  std::ofstream file(filename);

  if (!file.is_open()) {
    RCLCPP_ERROR(get_logger(), "Cannot open %s", filename.c_str());

    return false;
  }

  file << output;

  file.close();

  RCLCPP_INFO(get_logger(), "Saved %s", filename.c_str());

  return true;
}

// --------------------------------------------------
// COMMAND
// --------------------------------------------------

std::string CameraSettingsServer::run_command(const std::string &command) {

  std::array<char, 256> buffer;

  std::string output;

  FILE *pipe = popen(command.c_str(), "r");

  if (!pipe)
    return "";

  while (fgets(buffer.data(), buffer.size(), pipe)) {
    output += buffer.data();
  }

  pclose(pipe);

  return output;
}

// --------------------------------------------------
// PARSE V4L2
// --------------------------------------------------

std::unordered_map<std::string, int>
CameraSettingsServer::parse_controls(const std::string &output) {

  std::unordered_map<std::string, int> controls;

  std::regex pattern(R"(^\s*([a-zA-Z0-9_]+).*value=(-?\d+))");

  std::stringstream stream(output);

  std::string line;

  while (std::getline(stream, line)) {

    std::smatch match;

    if (std::regex_search(line, match, pattern)) {

      controls[match[1].str()] = std::stoi(match[2].str());
    }
  }

  return controls;
}

// --------------------------------------------------
// GET LIVE CAMERA
// --------------------------------------------------

void CameraSettingsServer::handle_request(
    const std::shared_ptr<GetRequest> request,
    std::shared_ptr<GetResponse> response) {

  response->success = false;

  auto camera = cameras_.find(request->camera);

  if (camera == cameras_.end())
    return;

  auto output =
      run_command("v4l2-ctl -d " + camera->second.path + " --list-ctrls");

  auto controls = parse_controls(output);

  if (controls.empty())
    return;

  camera->second.current_values = controls;

  fill_response(controls, *response);

  response->success = true;
}

// --------------------------------------------------
// FILL LIVE RESPONSE
// --------------------------------------------------

void CameraSettingsServer::fill_response(
    const std::unordered_map<std::string, int> &controls,
    GetResponse &response) {

  auto get = [&](const std::string &name) -> int {
    auto it = controls.find(name);

    if (it != controls.end()) {
      return it->second;
    }

    return 0;
  };

  response.backlight_compenstation = get("backlight_compensation");

  response.auto_exposure = get("auto_exposure");

  response.brightness = get("brightness");

  response.contrast = get("contrast");

  response.hue = get("hue");

  response.saturation = get("saturation");

  response.exposure_time_absolute = get("exposure_time_absolute");

  response.gain = get("gain");

  response.white_balance_automatic = get("white_balance_automatic") != 0;

  response.white_balance_temperature = get("white_balance_temperature");

  response.exposure_dynamic_framerate = get("exposure_dynamic_framerate") != 0;

  response.gamma = get("gamma");

  response.power_line_frequency = get("power_line_frequency");

  response.sharpness = get("sharpness");
}

// --------------------------------------------------
// GET YAML
// --------------------------------------------------

void CameraSettingsServer::get_from_yaml(
    const std::shared_ptr<GetRequest> request,
    std::shared_ptr<GetResponse> response) {
  response->success = false;

  std::filesystem::path config_path(config_file_);

  std::string filename =
      (config_path.parent_path() / (request->camera + "_settings.yaml"))
          .string();

  YAML::Node config;

  try {
    config = YAML::LoadFile(filename);
  } catch (const std::exception &e) {
    RCLCPP_WARN(get_logger(), "Failed loading preset %s", filename.c_str());

    return;
  }

  std::unordered_map<std::string, YAML::Node> controls;

  for (auto item : config) {
    controls[item.first.as<std::string>()] = item.second;
  }

  fill_yaml_response(controls, *response);

  response->success = true;
}

void CameraSettingsServer::fill_yaml_response(
    const std::unordered_map<std::string, YAML::Node> &controls,
    GetResponse &response) {

  auto get = [&](const std::string &name) -> int {
    auto it = controls.find(name);

    if (it != controls.end()) {
      if (it->second["value"]) {
        return it->second["value"].as<int>();
      }
    }

    return 0;
  };

  response.backlight_compenstation = get("backlight_compensation");

  response.auto_exposure = get("auto_exposure");

  response.brightness = get("brightness");

  response.contrast = get("contrast");

  response.hue = get("hue");

  response.saturation = get("saturation");

  response.exposure_time_absolute = get("exposure_time_absolute");

  response.gain = get("gain");

  response.white_balance_temperature = get("white_balance_temperature");

  response.gamma = get("gamma");

  response.power_line_frequency = get("power_line_frequency");

  response.sharpness = get("sharpness");

  response.white_balance_automatic = get("white_balance_automatic") != 0;

  response.exposure_dynamic_framerate = get("exposure_dynamic_framerate") != 0;
}

// --------------------------------------------------
// SET CAMERA
// --------------------------------------------------

void CameraSettingsServer::set_camera(const std::shared_ptr<SetRequest> request,
                                      std::shared_ptr<SetResponse> response) {

  response->success = false;

  auto camera = cameras_.find(request->camera);

  if (camera == cameras_.end())
    return;

  std::vector<std::string> changes;

  auto add = [&](std::string name, int value) {
    if (camera->second.controls.count(name)) {
      changes.push_back(name + "=" + std::to_string(value));
    }
  };

  add("backlight_compensation", request->backlight_compenstation);

  add("auto_exposure", request->auto_exposure);

  add("brightness", request->brightness);

  add("contrast", request->contrast);

  add("hue", request->hue);

  add("saturation", request->saturation);

  add("exposure_time_absolute", request->exposure_time_absolute);

  add("gain", request->gain);

  add("white_balance_temperature", request->white_balance_temperature);

  add("gamma", request->gamma);

  add("power_line_frequency", request->power_line_frequency);

  add("sharpness", request->sharpness);

  if (changes.empty()) {
    response->success = true;
    return;
  }

  std::string command = "v4l2-ctl -d " + camera->second.path + " -c ";

  for (size_t i = 0; i < changes.size(); i++) {
    command += changes[i];

    if (i + 1 < changes.size())
      command += ",";
  }

  int result = system(command.c_str());

  if (result == 0) {

    response->success = true;
  }
}

// --------------------------------------------------
// SAVE YAML
// --------------------------------------------------

void CameraSettingsServer::save_camera(
    const std::shared_ptr<SetRequest> request,
    std::shared_ptr<SetResponse> response) {

  response->success = false;

  auto camera = cameras_.find(request->camera);

  if (camera == cameras_.end())
    return;

  auto &values = camera->second.current_values;

  values["backlight_compensation"] = request->backlight_compenstation;

  values["auto_exposure"] = request->auto_exposure;

  values["brightness"] = request->brightness;

  values["contrast"] = request->contrast;

  values["hue"] = request->hue;

  values["saturation"] = request->saturation;

  values["exposure_time_absolute"] = request->exposure_time_absolute;

  values["gain"] = request->gain;

  values["white_balance_temperature"] = request->white_balance_temperature;

  values["gamma"] = request->gamma;

  values["power_line_frequency"] = request->power_line_frequency;

  values["sharpness"] = request->sharpness;

  response->success = save_config(request->camera);
}

} // namespace video_streaming

RCLCPP_COMPONENTS_REGISTER_NODE(video_streaming::CameraSettingsServer)