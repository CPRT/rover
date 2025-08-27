#ifndef MEMORY_COLLECTOR_HPP_
#define MEMORY_COLLECTOR_HPP_

#include <fstream>

#include "telemetry_collector.hpp"

class MemoryCollector : public TelemetryCollector {
 public:
  MemoryCollector();
  void collect(interfaces::msg::SystemTelemetry& msg) override;

 private:
  std::ifstream meminfo_file_;
};

#endif  // MEMORY_COLLECTOR_HPP_
