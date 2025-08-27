#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <vector>

#include "cpu_collector.hpp"
#include "gpu_collector.hpp"
#include "interfaces/msg/system_telemetry.hpp"
#include "memory_collector.hpp"
#include "telemetry_collector.hpp"

using namespace std::chrono_literals;

class SystemTelemetryPublisher : public rclcpp::Node {
 public:
  SystemTelemetryPublisher();

 private:
  void publish_telemetry();

  rclcpp::Publisher<interfaces::msg::SystemTelemetry>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<std::unique_ptr<TelemetryCollector>> collectors_;
};
