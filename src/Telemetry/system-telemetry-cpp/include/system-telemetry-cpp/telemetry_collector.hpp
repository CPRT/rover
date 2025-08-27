#ifndef TELEMETRY_COLLECTOR_HPP_
#define TELEMETRY_COLLECTOR_HPP_

#include <rclcpp/rclcpp.hpp>

#include "interfaces/msg/system_telemetry.hpp"

class TelemetryCollector {
 public:
  virtual void collect(interfaces::msg::SystemTelemetry& msg) = 0;
  virtual ~TelemetryCollector() = default;

 protected:
  rclcpp::Logger logger_ = rclcpp::get_logger("telemetry_collector");
};

#endif  // TELEMETRY_COLLECTOR_HPP_
