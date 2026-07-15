#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class SnmpNetworkStatsNode : public rclcpp::Node {
public:
  SnmpNetworkStatsNode();

private:
  void poll_and_publish();

  std::optional<int> snmp_get_int(const std::string &oid);
  std::optional<int> snmp_walk_first_int(const std::string &base_oid);

  void publish_float(
      const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr &publisher,
      float value);

  std::optional<float> compute_throughput_bps(uint32_t current_counter,
                                              uint32_t previous_counter,
                                              double dt_sec) const;

  std::string ip_address_;
  double frequency_;
  std::string community_;
  int snmp_version_;
  long timeout_us_;
  int retries_;

  const std::string oid_signal_strength_base_ =
      "iso.3.6.1.4.1.41112.1.4.5.1.5.1";
  const std::string oid_noise_floor_base_ = "iso.3.6.1.4.1.41112.1.4.5.1.8.1";
  const std::string oid_ccq_tx_base_ = "iso.3.6.1.4.1.41112.1.4.5.1.7.1";
  const std::string oid_bandwidth_tx_base_ = "iso.3.6.1.4.1.41112.1.4.5.1.9.1";
  const std::string oid_bandwidth_rx_base_ = "iso.3.6.1.4.1.41112.1.4.5.1.10.1";

  const std::string oid_rx_counter_ = "iso.3.6.1.2.1.2.2.1.10.6";
  const std::string oid_tx_counter_ = "iso.3.6.1.2.1.2.2.1.16.6";

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_bandwidth_tx_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_bandwidth_rx_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_throughput_tx_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_throughput_rx_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_signal_strength_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_noise_floor_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_ccq_tx_;

  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<uint32_t> prev_rx_counter_;
  std::optional<uint32_t> prev_tx_counter_;
  std::optional<std::chrono::steady_clock::time_point> prev_time_;
};