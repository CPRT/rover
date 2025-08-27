#include "system_telemetry_publisher.hpp"

#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "cpu_collector.hpp"
#include "gpu_collector.hpp"
#include "interfaces/msg/system_telemetry.hpp"
#include "memory_collector.hpp"
#include "telemetry_collector.hpp"

SystemTelemetryPublisher::SystemTelemetryPublisher()
    : Node("system_telemetry_publisher") {
  publisher_ = this->create_publisher<interfaces::msg::SystemTelemetry>(
      "/system_telemetry", 10);

  // declare parameters
  this->declare_parameter("frequency", 1.0);
  const double frequency = this->get_parameter("frequency").as_double();
  const auto period = std::chrono::duration<double>(1.0 / frequency);
  this->declare_parameter("cpu", true);
  this->declare_parameter("memory", true);
  this->declare_parameter("gpu", true);
  const bool cpu = this->get_parameter("cpu").as_bool();
  const bool memory = this->get_parameter("memory").as_bool();
  const bool gpu = this->get_parameter("gpu").as_bool();

  if (!cpu && !memory && !gpu) {
    RCLCPP_ERROR(this->get_logger(), "No collectors enabled.");
    return;
  }
  if (cpu) {
    collectors_.emplace_back(std::make_unique<CPUCollector>());
  }
  if (memory) {
    collectors_.emplace_back(std::make_unique<MemoryCollector>());
  }
  if (gpu) {
    collectors_.emplace_back(std::make_unique<GPUCollector>());
  }

  timer_ = this->create_wall_timer(
      period, std::bind(&SystemTelemetryPublisher::publish_telemetry, this));
  RCLCPP_INFO(this->get_logger(), "System Telemetry Publisher started.");
}

void SystemTelemetryPublisher::publish_telemetry() {
  interfaces::msg::SystemTelemetry msg;
  for (const auto& collector : collectors_) {
    collector->collect(msg);
  }
  publisher_->publish(msg);
  RCLCPP_INFO(this->get_logger(),
              "Published Telemetry: CPU: %.1f%%, Mem: %.1f%%, GPU: %.1f%%",
              msg.cpu_usage, msg.mem_usage, msg.gpu_usage);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SystemTelemetryPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
