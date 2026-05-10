#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <zlib.h>

#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace compressed_telemetry_cpp {

namespace {
std::vector<uint8_t> CompressZlib(const uint8_t *data, size_t size) {
  uLongf compressed_size = compressBound(size);
  std::vector<uint8_t> compressed(compressed_size);
  int ret = compress2(compressed.data(), &compressed_size,
                      reinterpret_cast<const Bytef *>(data), size,
                      Z_DEFAULT_COMPRESSION);
  if (ret != Z_OK) {
    throw std::runtime_error("zlib compression failed");
  }
  compressed.resize(compressed_size);
  return compressed;
}
} // namespace

class CostmapCompressor : public rclcpp::Node {
public:
  explicit CostmapCompressor(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("costmap_compressor", options),
        log_mbps_stats_(this->declare_parameter<bool>("log_mbps_stats", true)),
        raw_bytes_window_(0), compressed_bytes_window_(0),
        message_count_window_(0), received_count_window_(0) {
    auto radio_qos = rclcpp::QoS(1).best_effort().durability_volatile();
    auto sensor_qos = rclcpp::SensorDataQoS();

    full_map_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/telemetry/costmap_full_compressed", radio_qos);
    update_pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/telemetry/costmap_updates_compressed", radio_qos);

    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/global_costmap/costmap", sensor_qos,
        std::bind(&CostmapCompressor::mapCallback, this,
                  std::placeholders::_1));

    update_sub_ = this->create_subscription<map_msgs::msg::OccupancyGridUpdate>(
        "/global_costmap/costmap_updates", sensor_qos,
        std::bind(&CostmapCompressor::updateCallback, this,
                  std::placeholders::_1));

    stats_timer_ = this->create_wall_timer(
        std::chrono::seconds(20),
        std::bind(&CostmapCompressor::statsTimerCallback, this));

    map_thread_ = std::thread(&CostmapCompressor::mapProcessingLoop, this);
    update_thread_ =
        std::thread(&CostmapCompressor::updateProcessingLoop, this);

    RCLCPP_INFO(this->get_logger(), "Costmap Compressor Node Started.");
  }

  ~CostmapCompressor() override {
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      stop_processing_ = true;
    }
    map_cv_.notify_all();
    update_cv_.notify_all();
    if (map_thread_.joinable()) {
      map_thread_.join();
    }
    if (update_thread_.joinable()) {
      update_thread_.join();
    }
  }

private:
  void updateCallback(const map_msgs::msg::OccupancyGridUpdate::SharedPtr msg) {
    received_count_window_.fetch_add(1, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(update_mutex_);
      latest_update_ = msg;
    }
    update_cv_.notify_one();
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    received_count_window_.fetch_add(1, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      latest_full_map_ = msg;
    }
    map_cv_.notify_one();
  }

  void mapProcessingLoop() {
    while (rclcpp::ok()) {
      nav_msgs::msg::OccupancyGrid::SharedPtr current;
      {
        std::unique_lock<std::mutex> lock(map_mutex_);
        map_cv_.wait(lock,
                     [&]() { return stop_processing_ || latest_full_map_; });
        if (stop_processing_) {
          break;
        }
        current = std::move(latest_full_map_);
        latest_full_map_.reset();
      }

      if (!current) {
        continue;
      }

      try {
        rclcpp::SerializedMessage serialized_msg;
        serializer_map_.serialize_message(current.get(), &serialized_msg);
        auto &rcl_serialized = serialized_msg.get_rcl_serialized_message();
        auto compressed =
            CompressZlib(rcl_serialized.buffer, rcl_serialized.buffer_length);

        raw_bytes_window_.fetch_add(rcl_serialized.buffer_length,
                                    std::memory_order_relaxed);
        compressed_bytes_window_.fetch_add(compressed.size(),
                                           std::memory_order_relaxed);
        message_count_window_.fetch_add(1, std::memory_order_relaxed);

        std_msgs::msg::UInt8MultiArray out_msg;
        out_msg.data = std::move(compressed);
        full_map_pub_->publish(out_msg);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to compress full map: %s",
                     e.what());
      }
    }
  }

  void updateProcessingLoop() {
    while (rclcpp::ok()) {
      map_msgs::msg::OccupancyGridUpdate::SharedPtr current;
      {
        std::unique_lock<std::mutex> lock(update_mutex_);
        update_cv_.wait(lock,
                        [&]() { return stop_processing_ || latest_update_; });
        if (stop_processing_) {
          break;
        }
        current = std::move(latest_update_);
        latest_update_.reset();
      }

      if (!current) {
        continue;
      }

      try {
        rclcpp::SerializedMessage serialized_msg;
        serializer_update_.serialize_message(current.get(), &serialized_msg);
        auto &rcl_serialized = serialized_msg.get_rcl_serialized_message();
        auto compressed =
            CompressZlib(rcl_serialized.buffer, rcl_serialized.buffer_length);

        raw_bytes_window_.fetch_add(rcl_serialized.buffer_length,
                                    std::memory_order_relaxed);
        compressed_bytes_window_.fetch_add(compressed.size(),
                                           std::memory_order_relaxed);
        message_count_window_.fetch_add(1, std::memory_order_relaxed);

        std_msgs::msg::UInt8MultiArray out_msg;
        out_msg.data = std::move(compressed);
        update_pub_->publish(out_msg);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to compress update: %s",
                     e.what());
      }
    }
  }

  void statsTimerCallback() {
    if (!log_mbps_stats_) {
      return;
    }

    constexpr double duration_s = 20.0;
    const double compressed = static_cast<double>(
        compressed_bytes_window_.exchange(0, std::memory_order_relaxed));
    const double raw = static_cast<double>(
        raw_bytes_window_.exchange(0, std::memory_order_relaxed));
    const double sent_count = static_cast<double>(
        message_count_window_.exchange(0, std::memory_order_relaxed));
    const double received_count = static_cast<double>(
        received_count_window_.exchange(0, std::memory_order_relaxed));
    const double mbps = (compressed * 8.0) / (1e6 * duration_s);
    const double recv_hz = received_count / duration_s;
    const double sent_hz = sent_count / duration_s;

    if (sent_count > 0.0 && raw > 0.0) {
      const double avg_raw = raw / sent_count;
      const double avg_compressed = compressed / sent_count;
      const double avg_ratio = compressed / raw;
      const double avg_reduction = (1.0 - avg_ratio) * 100.0;
      RCLCPP_INFO(this->get_logger(),
                  "Avg compression over %d s: %.1f%% (avg %.0f bytes -> %.0f "
                  "bytes), %.3f Mbps, incoming %.2f Hz, compressed %.2f Hz",
                  static_cast<int>(duration_s), avg_reduction, avg_raw,
                  avg_compressed, mbps, recv_hz, sent_hz);
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Avg compression over %d s: no data (%.3f Mbps, incoming "
                  "%.2f Hz, compressed %.2f Hz)",
                  static_cast<int>(duration_s), mbps, recv_hz, sent_hz);
    }

    (void)sent_count;
  }

  rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> serializer_map_;
  rclcpp::Serialization<map_msgs::msg::OccupancyGridUpdate> serializer_update_;

  bool log_mbps_stats_;
  std::atomic<uint64_t> raw_bytes_window_;
  std::atomic<uint64_t> compressed_bytes_window_;
  std::atomic<uint64_t> message_count_window_;
  std::atomic<uint64_t> received_count_window_;

  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::mutex map_mutex_;
  std::mutex update_mutex_;
  std::condition_variable map_cv_;
  std::condition_variable update_cv_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_full_map_;
  map_msgs::msg::OccupancyGridUpdate::SharedPtr latest_update_;
  std::thread map_thread_;
  std::thread update_thread_;
  std::atomic<bool> stop_processing_{false};

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr full_map_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr update_pub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<map_msgs::msg::OccupancyGridUpdate>::SharedPtr
      update_sub_;
};

} // namespace compressed_telemetry_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(compressed_telemetry_cpp::CostmapCompressor)
