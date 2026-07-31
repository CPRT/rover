#ifndef VIDEO_STREAMING__CAMERA_SETTINGS_SERVER_HPP_
#define VIDEO_STREAMING__CAMERA_SETTINGS_SERVER_HPP_

#include <filesystem>
#include <memory>
#include <string>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "interfaces/srv/get_camera.hpp"
#include "interfaces/srv/set_camera.hpp"

namespace video_streaming {

class CameraSettingsServer : public rclcpp::Node {

public:
  using GetCamera = interfaces::srv::GetCamera;
  using GetRequest = GetCamera::Request;
  using GetResponse = GetCamera::Response;

  using SetCamera = interfaces::srv::SetCamera;
  using SetRequest = SetCamera::Request;
  using SetResponse = SetCamera::Response;
  explicit CameraSettingsServer(const rclcpp::NodeOptions &options);

private:
  struct CameraInfo {

    std::string path;

    // YAML controls
    std::unordered_map<std::string, YAML::Node> controls;

    // Last live values
    std::unordered_map<std::string, int> current_values;
  };

  //
  // YAML
  //

  void load_config(const std::string &filename);

  bool save_config(const std::string &camera_name);

  //
  // Services
  //

  // Read live camera
  void handle_request(const std::shared_ptr<GetRequest> request,
                      std::shared_ptr<GetResponse> response);

  // Read YAML saved values
  void get_from_yaml(const std::shared_ptr<GetRequest> request,
                     std::shared_ptr<GetResponse> response);

  // Write camera
  void set_camera(const std::shared_ptr<SetRequest> request,
                  std::shared_ptr<SetResponse> response);

  // Write YAML
  void save_camera(const std::shared_ptr<SetRequest> request,
                   std::shared_ptr<SetResponse> response);

  //
  // Helpers
  //

  std::string run_command(const std::string &command);

  std::unordered_map<std::string, int>
  parse_controls(const std::string &output);

  void fill_response(const std::unordered_map<std::string, int> &controls,
                     GetResponse &response);

  void fill_yaml_response(
      const std::unordered_map<std::string, YAML::Node> &controls,
      GetResponse &response);

  //
  // Data
  //

  std::unordered_map<std::string, CameraInfo> cameras_;

  //
  // Services
  //

  rclcpp::Service<GetCamera>::SharedPtr service_;

  rclcpp::Service<GetCamera>::SharedPtr yaml_service_;

  rclcpp::Service<SetCamera>::SharedPtr set_service_;

  rclcpp::Service<SetCamera>::SharedPtr save_service_;

  //
  // Config path
  //

  std::string config_file_;
};

} // namespace video_streaming

#endif