#pragma once
#include "base_video_node.hpp"
#include <gst/gst.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"  //bitrate
#include "std_msgs/msg/empty.hpp"  //I-frame



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
  
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // rclcpp::Publisher<video_streaming::msg::Srtstat>::SharedPtr stats_pub_;

private:

  void on_bitrate_received(const std_msgs::msg::Int32::SharedPtr msg);

  void on_iframe_trigger(const std_msgs::msg::Empty::SharedPtr msg);

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter> &parameters);


  /* =========================================================
   * TODO: FUTURE FEATURE - SRT STATISTICS
   * Uncomment these when Srtstat.msg is available.
   * =========================================================
   */
  // static void on_srt_stats(GstElement * element, GstStructure * stats, gpointer user_data);
  // rclcpp::Publisher<video_streaming::msg::Srtstat>::SharedPtr stats_pub_;

};

} 