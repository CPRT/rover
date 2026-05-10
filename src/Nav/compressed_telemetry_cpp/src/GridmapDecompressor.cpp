#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <zlib.h>

#include "grid_map_msgs/msg/grid_map.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "std_msgs/msg/multi_array_layout.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace compressed_telemetry_cpp {

namespace {
constexpr uint8_t kProtocolVersion = 0x01;
constexpr size_t kHeaderSize =
    2 + (10 * sizeof(double)) + (2 * sizeof(uint16_t));

std::vector<uint8_t> DecompressZlib(const uint8_t *data, size_t size) {
  constexpr size_t kMaxDecompressed = 100 * 1024 * 1024;
  size_t output_size = std::max<size_t>(size * 4, 1024);

  for (int attempt = 0; attempt < 8 && output_size <= kMaxDecompressed;
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
      throw std::runtime_error("zlib decompression failed");
    }
    output_size *= 2;
  }

  throw std::runtime_error("zlib decompression exceeded size limits");
}

template <typename T>
T ReadLittleEndian(const std::vector<uint8_t> &buffer, size_t offset) {
  T value{};
  std::memcpy(&value, buffer.data() + offset, sizeof(T));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
  uint8_t *raw = reinterpret_cast<uint8_t *>(&value);
  std::reverse(raw, raw + sizeof(T));
#endif
  return value;
}
} // namespace

class GridmapDecompressor : public rclcpp::Node {
public:
  explicit GridmapDecompressor(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : rclcpp::Node("gridmap_decompressor", options) {
    auto radio_qos = rclcpp::QoS(1).best_effort().durability_volatile();
    auto rviz_qos = rclcpp::QoS(1).reliable().transient_local();

    pub_ = this->create_publisher<grid_map_msgs::msg::GridMap>(
        "/viz/traversability_map", rviz_qos);
    sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/telemetry/gridmap_compressed", radio_qos,
        std::bind(&GridmapDecompressor::mapCallback, this,
                  std::placeholders::_1));
  }

private:
  void mapCallback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    try {
      auto raw_payload = DecompressZlib(msg->data.data(), msg->data.size());
      if (raw_payload.size() < kHeaderSize) {
        RCLCPP_WARN(this->get_logger(),
                    "Dropped gridmap packet: payload smaller than header.");
        return;
      }

      uint8_t protocol_version = raw_payload[0];
      if (protocol_version != kProtocolVersion) {
        RCLCPP_WARN(
            this->get_logger(),
            "Dropped gridmap packet: protocol version mismatch (%u != %u).",
            protocol_version, kProtocolVersion);
        return;
      }

      const uint8_t frame_len = raw_payload[1];
      size_t offset = 2;

      const double resolution = ReadLittleEndian<double>(raw_payload, offset);
      offset += sizeof(double);
      const double length_x = ReadLittleEndian<double>(raw_payload, offset);
      offset += sizeof(double);
      const double length_y = ReadLittleEndian<double>(raw_payload, offset);
      offset += sizeof(double);

      const double px = ReadLittleEndian<double>(raw_payload, offset);
      offset += sizeof(double);
      const double py = ReadLittleEndian<double>(raw_payload, offset);
      offset += sizeof(double);
      const double pz = ReadLittleEndian<double>(raw_payload, offset);
      offset += sizeof(double);

      const double qx = ReadLittleEndian<double>(raw_payload, offset);
      offset += sizeof(double);
      const double qy = ReadLittleEndian<double>(raw_payload, offset);
      offset += sizeof(double);
      const double qz = ReadLittleEndian<double>(raw_payload, offset);
      offset += sizeof(double);
      const double qw = ReadLittleEndian<double>(raw_payload, offset);
      offset += sizeof(double);

      const uint16_t outer_start =
          ReadLittleEndian<uint16_t>(raw_payload, offset);
      offset += sizeof(uint16_t);
      const uint16_t inner_start =
          ReadLittleEndian<uint16_t>(raw_payload, offset);
      offset += sizeof(uint16_t);

      if (raw_payload.size() < offset + frame_len) {
        RCLCPP_WARN(this->get_logger(),
                    "Dropped gridmap packet: truncated frame_id bytes.");
        return;
      }

      const std::string frame_id(
          reinterpret_cast<const char *>(raw_payload.data() + offset),
          frame_len);
      offset += frame_len;

      const size_t remaining_bytes = raw_payload.size() - offset;
      if (remaining_bytes % 3 != 0) {
        RCLCPP_WARN(this->get_logger(),
                    "Dropped gridmap packet: payload alignment is invalid.");
        return;
      }

      const size_t num_cells = remaining_bytes / 3;
      const size_t elev_bytes_len = num_cells * sizeof(int16_t);
      if (remaining_bytes < elev_bytes_len) {
        RCLCPP_WARN(this->get_logger(),
                    "Dropped gridmap packet: elevation bytes missing.");
        return;
      }

      const uint8_t *elev_bytes = raw_payload.data() + offset;
      const uint8_t *trav_bytes = raw_payload.data() + offset + elev_bytes_len;

      std::vector<float> elev_float(num_cells);
      std::vector<float> trav_float(num_cells);

      for (size_t i = 0; i < num_cells; ++i) {
        int16_t elev_val;
        std::memcpy(&elev_val, elev_bytes + i * sizeof(int16_t),
                    sizeof(int16_t));
#if __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__
        elev_val = static_cast<int16_t>((static_cast<uint16_t>(elev_val) >> 8) |
                                        (static_cast<uint16_t>(elev_val) << 8));
#endif
        if (elev_val == -32768) {
          elev_float[i] = std::numeric_limits<float>::quiet_NaN();
        } else {
          elev_float[i] = static_cast<float>(elev_val) / 1000.0f;
        }

        const uint8_t trav_val = trav_bytes[i];
        if (trav_val == 255) {
          trav_float[i] = std::numeric_limits<float>::quiet_NaN();
        } else {
          trav_float[i] = static_cast<float>(trav_val) / 100.0f;
        }
      }

      grid_map_msgs::msg::GridMap grid_msg;
      const auto now = this->get_clock()->now();
      const auto now_ns = now.nanoseconds();
      grid_msg.header.stamp.sec = static_cast<int32_t>(now_ns / 1000000000LL);
      grid_msg.header.stamp.nanosec =
          static_cast<uint32_t>(now_ns % 1000000000LL);
      grid_msg.header.frame_id = frame_id;

      grid_msg.info.resolution = resolution;
      grid_msg.info.length_x = length_x;
      grid_msg.info.length_y = length_y;
      grid_msg.info.pose.position.x = px;
      grid_msg.info.pose.position.y = py;
      grid_msg.info.pose.position.z = pz;
      grid_msg.info.pose.orientation.x = qx;
      grid_msg.info.pose.orientation.y = qy;
      grid_msg.info.pose.orientation.z = qz;
      grid_msg.info.pose.orientation.w = qw;

      grid_msg.layers = {"elevation", "traversability"};
      grid_msg.basic_layers = {};
      grid_msg.outer_start_index = outer_start;
      grid_msg.inner_start_index = inner_start;

      int size_x =
          std::max(1, static_cast<int>(std::round(length_x / resolution)));
      int size_y =
          std::max(1, static_cast<int>(std::round(length_y / resolution)));

      if (static_cast<size_t>(size_x) * static_cast<size_t>(size_y) !=
          num_cells) {
        RCLCPP_WARN(this->get_logger(),
                    "Gridmap dimensions do not match payload size (%zu cells "
                    "vs %d x %d).",
                    num_cells, size_x, size_y);
      }

      std_msgs::msg::MultiArrayLayout layout;
      layout.dim.resize(2);
      layout.dim[0].label = "column_index";
      layout.dim[0].size = static_cast<uint32_t>(size_x);
      layout.dim[0].stride = static_cast<uint32_t>(size_y);
      layout.dim[1].label = "row_index";
      layout.dim[1].size = static_cast<uint32_t>(size_y);
      layout.dim[1].stride = 1;
      layout.data_offset = 0;

      std_msgs::msg::Float32MultiArray elev_multi;
      elev_multi.layout = layout;
      elev_multi.data = std::move(elev_float);

      std_msgs::msg::Float32MultiArray trav_multi;
      trav_multi.layout = layout;
      trav_multi.data = std::move(trav_float);

      grid_msg.data = {std::move(elev_multi), std::move(trav_multi)};

      pub_->publish(grid_msg);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Decompression failed: %s", e.what());
    }
  }

  rclcpp::Publisher<grid_map_msgs::msg::GridMap>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;
};

} // namespace compressed_telemetry_cpp

RCLCPP_COMPONENTS_REGISTER_NODE(compressed_telemetry_cpp::GridmapDecompressor)
