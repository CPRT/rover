#pragma once
#include "base_video_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp" // I-frame
#include "std_msgs/msg/int32.hpp" // bitrate
#include <chrono>                 // timers
#include <gst/gst.h>
#include <interfaces/msg/srt_stats.hpp>

namespace video_streaming {

class SrtNode : public BaseVideoNode {
public:
  explicit SrtNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  ~SrtNode() override;

  static constexpr int INITIAL_BACKOFF_MS = 200;
  static constexpr int MAX_BACKOFF_MS = 5000;
  static constexpr int BACKOFF_RESET_MS = 10000;

protected:
  // (interpipesrc -> nvvidconv -> nvv4l2av1enc -> srtsink)
  bool create_pipeline() override;

  GstElement *av1_encoder_ = nullptr;
  GstElement *srt_sink_ = nullptr;
  GstElement *framerate_caps_ = nullptr;

  // bitrate
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bitrate_sub_;

  // I-frame
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr iframe_sub_;

  // Parameter callback handle
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // SRT Stats publisher
  rclcpp::Publisher<interfaces::msg::SrtStats>::SharedPtr srt_stats_pub_;
  rclcpp::TimerBase::SharedPtr srt_stats_timer_;

private:
  void on_bitrate_received(const std_msgs::msg::Int32::SharedPtr msg);

  void on_iframe_trigger(const std_msgs::msg::Empty::SharedPtr msg);

  void publish_srt_stats();

  void change_framerate(int new_fps);

  rcl_interfaces::msg::SetParametersResult
  on_parameter_change(const std::vector<rclcpp::Parameter> &parameters);

  // Backoff mechanism to avoid excessive I-frame requests
  void check_packet_loss_and_trigger(int64_t current_total_dropped);

  struct BackoffState {
    int current_delay_ms = INITIAL_BACKOFF_MS;
    int max_delay_ms = MAX_BACKOFF_MS;
    int reset_ms = BACKOFF_RESET_MS;

    std::chrono::steady_clock::time_point last_trigger_time;
    std::chrono::steady_clock::time_point last_loss_time;
    int64_t last_total_dropped_pkts = 0;
  } backoff_state_;

  int iframe_interval_ = 0;
  int target_framerate_ = 30;
};

} // namespace video_streaming
