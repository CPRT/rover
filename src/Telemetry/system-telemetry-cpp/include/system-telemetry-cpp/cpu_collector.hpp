#ifndef CPU_COLLECTOR_HPP_
#define CPU_COLLECTOR_HPP_

#include <fstream>

#include "telemetry_collector.hpp"

class CPUCollector : public TelemetryCollector {
 public:
  CPUCollector();
  void collect(interfaces::msg::SystemTelemetry& msg) override;

 private:
  struct CPUStats {
    uint64_t total;
    uint64_t idle;
  };

  CPUStats read_cpu_stats();
  static double calculate_cpu_usage(const CPUStats& prev, const CPUStats& curr);

  std::ifstream stat_file_;
  CPUStats prev_stats_;
};

#endif  // CPU_COLLECTOR_HPP_
