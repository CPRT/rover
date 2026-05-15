#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
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
std::vector<uint8_t> DecompressZlib(const uint8_t *data, size_t size,
                                    size_t max_decompressed) {
  size_t output_size = std::max<size_t>(size * 4, 1024);

  for (int attempt = 0; attempt < 8 && output_size <= max_decompressed;
       ++attempt) {
    std::vector<uint8_t> output(output_size);
    uLongf dest_len = output_size;
    int ret = uncompress(reinterpret_cast<Bytef *>(output.data()), &dest_len,
                         reinterpret_cast<const Bytef *>(data), size);
    if (ret == Z_OK) {
      output.resize(dest_len);
      return output;
    }
    if (ret != Z_BUF_ERROR) {
      throw std::runtime_error("zlib decompression failed (code " +
                               std::to_string(ret) + ")");
    }
    output_size *= 2;
  }

  throw std::runtime_error(
      "zlib decompression exceeded size limits (compressed=" +
      std::to_string(size) + ", limit=" + std::to_string(max_decompressed) +
      ")");
}
} // namespace

class CostmapDecompressor : public rclcpp::Node {
public:
  explicit CostmapDecompressor(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("costmap_decompressor", options),
        max_decompressed_bytes_(
            static_cast<size_t>(this->declare_parameter<int64_t>(
                "max_decompressed_bytes", 100 * 1024 * 1024))) {
    if (max_decompressed_bytes_ == 0) {
      max_decompressed_bytes_ = 100 * 1024 * 1024;
      RCLCPP_WARN(this->get_logger(),
                  "max_decompressed_bytes must be > 0; using default %zu",
                  max_decompressed_bytes_);
    }
    auto radio_qos = rclcpp::QoS(1).best_effort().durability_volatile();
    auto rviz_qos = rclcpp::QoS(1).reliable().transient_local();

    full_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/viz/global_costmap/costmap", rviz_qos);
    update_pub_ = this->create_publisher<map_msgs::msg::OccupancyGridUpdate>(
        "/viz/global_costmap/costmap_updates", rclcpp::QoS(10));

    full_map_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/telemetry/costmap_full_compressed", radio_qos,
        std::bind(&CostmapDecompressor::fullMapCallback, this,
                  std::placeholders::_1));
    update_sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/telemetry/costmap_updates_compressed", radio_qos,
        std::bind(&CostmapDecompressor::updateCallback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Costmap Decompressor Node Started.");
  }

private:
  void fullMapCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    try {
      auto raw = DecompressZlib(msg->data.data(), msg->data.size(),
                                max_decompressed_bytes_);
      rclcpp::SerializedMessage serialized_msg(raw.size());
      auto &rcl_serialized = serialized_msg.get_rcl_serialized_message();
      std::memcpy(rcl_serialized.buffer, raw.data(), raw.size());
      rcl_serialized.buffer_length = raw.size();

      nav_msgs::msg::OccupancyGrid grid_msg;
      serializer_map_.deserialize_message(&serialized_msg, &grid_msg);
      const auto now = this->get_clock()->now();
      const auto now_ns = now.nanoseconds();
      grid_msg.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
      grid_msg.header.stamp.nanosec =
          static_cast<uint32_t>(now_ns % 1000000000LL);
      full_map_pub_->publish(grid_msg);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to unpack full map: %s",
                   e.what());
    }
  }

  void updateCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    try {
      auto raw = DecompressZlib(msg->data.data(), msg->data.size(),
                                max_decompressed_bytes_);
      rclcpp::SerializedMessage serialized_msg(raw.size());
      auto &rcl_serialized = serialized_msg.get_rcl_serialized_message();
      std::memcpy(rcl_serialized.buffer, raw.data(), raw.size());
      rcl_serialized.buffer_length = raw.size();

      map_msgs::msg::OccupancyGridUpdate update_msg;
      serializer_update_.deserialize_message(&serialized_msg, &update_msg);
      const auto now = this->get_clock()->now();
      const auto now_ns = now.nanoseconds();
      update_msg.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
      update_msg.header.stamp.nanosec =
          static_cast<uint32_t>(now_ns % 1000000000LL);
      update_pub_->publish(update_msg);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to unpack map update: %s",
                   e.what());
    }
  }

  rclcpp::Serialization<nav_msgs::msg::OccupancyGrid> serializer_map_;
  rclcpp::Serialization<map_msgs::msg::OccupancyGridUpdate> serializer_update_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr full_map_pub_;
  rclcpp::Publisher<map_msgs::msg::OccupancyGridUpdate>::SharedPtr update_pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr full_map_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr update_sub_;
  size_t max_decompressed_bytes_;
};

} // namespace compressed_telemetry_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(compressed_telemetry_cpp::CostmapDecompressor)
