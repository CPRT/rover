#ifndef GPU_COLLECTOR_HPP_
#define GPU_COLLECTOR_HPP_

#include "telemetry_collector.hpp"

class GPUCollector : public TelemetryCollector {
 public:
  void collect(interfaces::msg::SystemTelemetry& msg) override;

 private:
  double get_gpu_usage();
};

#endif  // GPU_COLLECTOR_HPP_
