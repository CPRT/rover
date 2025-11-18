#pragma once
#include "base_video_node.hpp"
#include <gst/gst.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"  //bitrate
#include "std_msgs/msg/empty.hpp"  //I-frame
#include "video_streaming/srv/set_streaming_params.hpp"
#include "video_streaming/msg/srtstat.hpp"


namespace video_streaming

{
class SrtNode : public BaseVideoNode {
public:
  explicit SrtNode(
            const rclcpp::NodeOptions &options = rclcpp::NodeOptions());            
  ~SrtNode() override = default;

protected:
  // (interpipesrc -> nvvidconv -> nvv4l2av1enc -> srtsink)
  bool create_pipeline() override;

  GstElement * av1_encoder_ = nullptr;
  GstElement * srt_sink_ = nullptr;


  //bitrate
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr bitrate_sub_;

  // I-frame
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr iframe_sub_;

  rclcpp::Publisher<video_streaming::msg::Srtstat>::SharedPtr stats_pub_;

  /**  (latency, I-frame interval, framerate) */
  rclcpp::Service<video_streaming::srv::SetStreamingParams>::SharedPtr params_srv_; 

private:

  void on_bitrate_received(const std_msgs::msg::Int32::SharedPtr msg);

  void on_iframe_trigger(const std_msgs::msg::Empty::SharedPtr msg);

  void on_set_params(
    const std::shared_ptr<video_streaming::srv::SetStreamingParams::Request> request,  
    std::shared_ptr<video_streaming::srv::SetStreamingParams::Response> response);


  static void on_srt_stats(GstElement * element, GstStructure * stats, gpointer user_data);
};

} 