#include "srt_node.hpp"
#include <rclcpp_components/register_node_macro.hpp>
#include <glib.h>

namespace video_streaming 
{

SrtNode::SrtNode(const rclcpp::NodeOptions &options)
  : BaseVideoNode("srt_node", options) 
{
    RCLCPP_INFO(this->get_logger(), "Initializing SrtNode...");

    // Start up
    this->declare_parameter<std::string>("srt_uri", "srt://127.0.0.1:12345");
    
    bitrate_sub_ = this->create_subscription<std_msgs::msg::Int32>(
        "~/set_bitrate", 10,
        std::bind(&SrtNode::on_bitrate_received, this, std::placeholders::_1));

    iframe_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "~/trigger_iframe", 10,
        std::bind(&SrtNode::on_iframe_trigger, this, std::placeholders::_1));

    stats_pub_ = this->create_publisher<video_streaming::msg::Srtstat>("~/srt_stats", 10);

    params_srv_ = this->create_service<video_streaming::srv::SetStreamingParams>(
        "~/set_params",
        std::bind(&SrtNode::on_set_params, this, std::placeholders::_1, std::placeholders::_2));

    if (!start_pipeline()) {
        RCLCPP_FATAL(get_logger(), "SrtNode: failed to start pipeline.");
        throw std::runtime_error("SrtNode: pipeline start failed");
    }
    RCLCPP_INFO(get_logger(), "SrtNode constructed and pipeline started.");
}


bool SrtNode::create_pipeline()
{
    std::string srt_uri = this->get_parameter("srt_uri").as_string();


    gchar *desc = g_strdup_printf(
        "interpipesrc listen-to=detect ! "
        "nvvidconv ! "
        "nvv4l2av1enc name=av1_enc ! "
        "rtpav1pay ! queue ! "
        "srtsink name=srt_sink uri=%s mode=1",
        srt_uri.c_str()
    );
    RCLCPP_INFO(this->get_logger(), "Using pipeline description: %s", desc);

    // same as webrtc_node.cpp
    GError *err = nullptr;
    GstElement *p = gst_parse_launch(desc, &err);
    g_free(desc); 

    if (err || !p) {
        RCLCPP_ERROR(get_logger(), "gst_parse_launch failed: %s",
                     err ? err->message : "unknown");
        if (err) g_error_free(err);
        return false;
    }
    if (!GST_IS_PIPELINE(p)) {
        RCLCPP_ERROR(get_logger(), "Parsed element is not a pipeline.");
        gst_object_unref(p);
        return false;
    }
    pipeline_ = p; 

    av1_encoder_ = this->get_element("av1_enc");
    srt_sink_ = this->get_element("srt_sink");

    if (!av1_encoder_ || !srt_sink_) {
        RCLCPP_ERROR(get_logger(), "Failed to get elements by name");
        return false;
    }

 
    g_signal_connect(srt_sink_, "get-stats", G_CALLBACK(SrtNode::on_srt_stats), this);

    RCLCPP_INFO(this->get_logger(), "Pipeline parsed and elements retrieved successfully.");
    return true;
}

 


void SrtNode::on_bitrate_received(const std_msgs::msg::Int32::SharedPtr msg)
{
    if (av1_encoder_) {
        g_object_set(av1_encoder_, "bitrate", msg->data, NULL);
        RCLCPP_INFO(this->get_logger(), "Bitrate set to %d", msg->data);
    }
}

void SrtNode::on_iframe_trigger(const std_msgs::msg::Empty::SharedPtr msg)
{
    (void)msg;
    if (av1_encoder_) {
        // nvv4l2av1enc  'force-key-unit' )
        g_signal_emit_by_name(av1_encoder_, "force-key-unit", NULL);
        RCLCPP_INFO(this->get_logger(), "I-Frame triggered");
    }
}

void SrtNode::on_set_params(
    const std::shared_ptr<video_streaming::srv::SetStreamingParams::Request> request,
    std::shared_ptr<video_streaming::srv::SetStreamingParams::Response> response)
{
    if (av1_encoder_ && srt_sink_) {
        g_object_set(av1_encoder_, "iframeinterval", request->iframe_interval, NULL);
        g_object_set(srt_sink_, "latency", request->latency, NULL);
        
        RCLCPP_INFO(this->get_logger(), "Parameters updated via service");
        response->success = true;
    } else {
        response->success = false;
        response->message = "Pipeline elements not available";
    }
}


void SrtNode::on_srt_stats(GstElement * element, GstStructure * stats, gpointer user_data)
{
    SrtNode* node = static_cast<SrtNode*>(user_data);
    auto stats_msg = video_streaming::msg::Srtstat();
    
    node->stats_pub_->publish(stats_msg);
}

} 

RCLCPP_COMPONENTS_REGISTER_NODE(video_streaming::SrtNode)
