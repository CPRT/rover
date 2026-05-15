#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "tf2_msgs/msg/tf_message.hpp"

namespace compressed_telemetry_cpp {

class TFTelemetry : public rclcpp::Node {
public:
  explicit TFTelemetry(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("tf_telemetry", options), received_count_window_(0),
        sent_count_window_(0) {

    // --- Parameters ---
    double publish_rate = this->declare_parameter<double>("publish_rate", 15.0);
    bool log_stats = this->declare_parameter<bool>("log_stats", true);

    // Allowed frames (empty means allow ALL frames)
    auto allowed_frames_vec = this->declare_parameter<std::vector<std::string>>(
        "allowed_frames", std::vector<std::string>());
    allowed_frames_ = std::unordered_set<std::string>(
        allowed_frames_vec.begin(), allowed_frames_vec.end());

    log_stats_ = log_stats;

    // --- QoS Settings ---
    // Best effort for the radio link
    auto radio_qos = rclcpp::QoS(10).best_effort().durability_volatile();
    auto local_qos = rclcpp::QoS(10);

    // --- Pub/Sub ---
    tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/telemetry/tf",
                                                               radio_qos);

    tf_sub_ = this->create_subscription<tf2_msgs::msg::TFMessage>(
        "/tf", local_qos,
        std::bind(&TFTelemetry::tfCallback, this, std::placeholders::_1));

    // --- Timers ---
    auto timer_period = std::chrono::duration<double>(1.0 / publish_rate);
    publish_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
        std::bind(&TFTelemetry::publishTimerCallback, this));

    stats_timer_ = this->create_wall_timer(
        std::chrono::seconds(20),
        std::bind(&TFTelemetry::statsTimerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "TF Telemetry Node Started. Publishing at %.1f Hz. Filtering "
                "%zu frames.",
                publish_rate, allowed_frames_.size());
  }

private:
  void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    received_count_window_.fetch_add(msg->transforms.size(),
                                     std::memory_order_relaxed);

    std::lock_guard<std::mutex> lock(cache_mutex_);
    for (const auto &transform : msg->transforms) {
      // If the allowed list is empty, pass everything.
      // Otherwise, check if the child_frame_id is allowed.
      if (allowed_frames_.empty() ||
          allowed_frames_.count(transform.child_frame_id) > 0) {
        // Overwriting guarantees we only keep the absolute latest transform
        tf_cache_[transform.child_frame_id] = transform;
      }
    }
  }

  void publishTimerCallback() {
    tf2_msgs::msg::TFMessage out_msg;

    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      if (tf_cache_.empty()) {
        return; // Nothing new to publish
      }

      for (auto &kv : tf_cache_) {
        out_msg.transforms.push_back(std::move(kv.second));
      }

      // Clear the cache so we don't send redundant data if nothing moved
      tf_cache_.clear();
    }

    // Calculate actual wire size
    rclcpp::SerializedMessage serialized_msg;
    tf_serializer_.serialize_message(&out_msg, &serialized_msg);
    total_bytes_window_.fetch_add(serialized_msg.size(),
                                  std::memory_order_relaxed);

    sent_count_window_.fetch_add(out_msg.transforms.size(),
                                 std::memory_order_relaxed);
    tf_pub_->publish(out_msg);
  }

  void statsTimerCallback() {
    if (!log_stats_) {
      return;
    }

    constexpr double duration_s = 20.0;
    const double bytes = static_cast<double>(
        total_bytes_window_.exchange(0, std::memory_order_relaxed));
    const double received = static_cast<double>(
        received_count_window_.exchange(0, std::memory_order_relaxed));
    const double sent = static_cast<double>(
        sent_count_window_.exchange(0, std::memory_order_relaxed));

    const double mbps = (bytes * 8.0) / (1e6 * duration_s);
    const double reduction =
        (received > 0.0) ? (1.0 - (sent / received)) * 100.0 : 0.0;

    RCLCPP_INFO(
        this->get_logger(),
        "TF Stats: %.1f%% drop | In: %.1f tf/s | Out: %.1f tf/s | BW: %.4f "
        "Mbps",
        reduction, received / duration_s, sent / duration_s, mbps);
  }

  // State
  std::unordered_set<std::string> allowed_frames_;
  std::unordered_map<std::string, geometry_msgs::msg::TransformStamped>
      tf_cache_;
  bool log_stats_;
  std::atomic<uint64_t> received_count_window_;
  std::atomic<uint64_t> sent_count_window_;
  std::atomic<uint64_t> total_bytes_window_{0};
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_serializer_;

  // Threading
  std::mutex cache_mutex_;

  // ROS
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr tf_sub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
};

} // namespace compressed_telemetry_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(compressed_telemetry_cpp::TFTelemetry)