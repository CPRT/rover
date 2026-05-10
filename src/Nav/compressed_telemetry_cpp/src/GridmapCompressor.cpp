#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <zlib.h>

#include "grid_map_msgs/msg/grid_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace compressed_telemetry_cpp {

namespace {
constexpr uint8_t kProtocolVersion = 0x01;

template <typename T>
void AppendLittleEndian(std::vector<uint8_t> &buffer, T value) {
  uint8_t raw[sizeof(T)];
  std::memcpy(raw, &value, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  std::reverse(raw, raw + sizeof(T));
#endif
  buffer.insert(buffer.end(), raw, raw + sizeof(T));
}

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

class GridmapCompressor : public rclcpp::Node {
public:
  explicit GridmapCompressor(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("gridmap_compressor", options),
        log_mbps_stats_(this->declare_parameter<bool>("log_mbps_stats", true)),
        raw_bytes_window_(0), compressed_bytes_window_(0),
        message_count_window_(0), received_count_window_(0) {
    auto radio_qos = rclcpp::QoS(1).best_effort().durability_volatile();
    auto sensor_qos = rclcpp::SensorDataQoS();

    pub_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "/telemetry/gridmap_compressed", radio_qos);
    sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
        "/traversability_map", sensor_qos,
        std::bind(&GridmapCompressor::mapCallback, this,
                  std::placeholders::_1));

    stats_timer_ = this->create_wall_timer(
        std::chrono::seconds(20),
        std::bind(&GridmapCompressor::statsTimerCallback, this));

    processing_thread_ = std::thread(&GridmapCompressor::processingLoop, this);
  }

  ~GridmapCompressor() override {
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      stop_processing_ = true;
    }
    map_cv_.notify_all();
    if (processing_thread_.joinable()) {
      processing_thread_.join();
    }
  }

private:
  void mapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg) {
    received_count_window_.fetch_add(1, std::memory_order_relaxed);
    {
      std::lock_guard<std::mutex> lock(map_mutex_);
      latest_map_ = msg;
    }
    map_cv_.notify_one();
  }

  void processingLoop() {
    while (rclcpp::ok()) {
      grid_map_msgs::msg::GridMap::SharedPtr current;
      {
        std::unique_lock<std::mutex> lock(map_mutex_);
        map_cv_.wait(lock, [&]() { return stop_processing_ || latest_map_; });
        if (stop_processing_) {
          break;
        }
        current = std::move(latest_map_);
        latest_map_.reset();
      }

      if (!current) {
        continue;
      }

      try {
        auto elev_it = std::find(current->layers.begin(), current->layers.end(),
                                 "elevation");
        auto trav_it = std::find(current->layers.begin(), current->layers.end(),
                                 "traversability");
        if (elev_it == current->layers.end() ||
            trav_it == current->layers.end()) {
          continue;
        }

        const size_t elev_idx = static_cast<size_t>(
            std::distance(current->layers.begin(), elev_it));
        const size_t trav_idx = static_cast<size_t>(
            std::distance(current->layers.begin(), trav_it));

        const auto &elev_data = current->data[elev_idx].data;
        const auto &trav_data = current->data[trav_idx].data;
        if (elev_data.size() != trav_data.size()) {
          RCLCPP_WARN(this->get_logger(), "Gridmap layer sizes do not match.");
          continue;
        }

        std::vector<int16_t> elev_int16(elev_data.size());
        std::vector<uint8_t> trav_uint8(trav_data.size());

        for (size_t i = 0; i < elev_data.size(); ++i) {
          const float elev_val = elev_data[i];
          if (std::isnan(elev_val)) {
            elev_int16[i] = -32768;
          } else {
            const float scaled =
                std::clamp(elev_val * 1000.0f, -32767.0f, 32767.0f);
            elev_int16[i] = static_cast<int16_t>(scaled);
          }

          const float trav_val = trav_data[i];
          if (std::isnan(trav_val)) {
            trav_uint8[i] = 255;
          } else {
            const float scaled = std::clamp(trav_val * 100.0f, 0.0f, 254.0f);
            trav_uint8[i] = static_cast<uint8_t>(scaled);
          }
        }

        const std::string frame_id = current->header.frame_id;
        if (frame_id.size() > 255) {
          RCLCPP_WARN(this->get_logger(),
                      "Frame id too long for gridmap packet.");
          continue;
        }

        std::vector<uint8_t> payload;
        payload.reserve(86 + frame_id.size() +
                        elev_int16.size() * sizeof(int16_t) +
                        trav_uint8.size());

        payload.push_back(kProtocolVersion);
        payload.push_back(static_cast<uint8_t>(frame_id.size()));

        AppendLittleEndian(payload, current->info.resolution);
        AppendLittleEndian(payload, current->info.length_x);
        AppendLittleEndian(payload, current->info.length_y);
        AppendLittleEndian(payload, current->info.pose.position.x);
        AppendLittleEndian(payload, current->info.pose.position.y);
        AppendLittleEndian(payload, current->info.pose.position.z);
        AppendLittleEndian(payload, current->info.pose.orientation.x);
        AppendLittleEndian(payload, current->info.pose.orientation.y);
        AppendLittleEndian(payload, current->info.pose.orientation.z);
        AppendLittleEndian(payload, current->info.pose.orientation.w);
        AppendLittleEndian(payload, current->outer_start_index);
        AppendLittleEndian(payload, current->inner_start_index);

        payload.insert(payload.end(), frame_id.begin(), frame_id.end());

        const uint8_t *elev_bytes =
            reinterpret_cast<const uint8_t *>(elev_int16.data());
        payload.insert(payload.end(), elev_bytes,
                       elev_bytes + elev_int16.size() * sizeof(int16_t));
        payload.insert(payload.end(), trav_uint8.begin(), trav_uint8.end());

        auto compressed = CompressZlib(payload.data(), payload.size());

        std_msgs::msg::UInt8MultiArray out_msg;
        out_msg.data = std::move(compressed);
        pub_->publish(out_msg);

        raw_bytes_window_.fetch_add(payload.size(), std::memory_order_relaxed);
        compressed_bytes_window_.fetch_add(out_msg.data.size(),
                                           std::memory_order_relaxed);
        message_count_window_.fetch_add(1, std::memory_order_relaxed);
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Compression failed: %s", e.what());
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

  bool log_mbps_stats_;
  std::atomic<uint64_t> raw_bytes_window_;
  std::atomic<uint64_t> compressed_bytes_window_;
  std::atomic<uint64_t> message_count_window_;
  std::atomic<uint64_t> received_count_window_;

  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub_;
  rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr stats_timer_;
  std::mutex map_mutex_;
  std::condition_variable map_cv_;
  grid_map_msgs::msg::GridMap::SharedPtr latest_map_;
  std::thread processing_thread_;
  std::atomic<bool> stop_processing_{false};
};

} // namespace compressed_telemetry_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(compressed_telemetry_cpp::GridmapCompressor)
