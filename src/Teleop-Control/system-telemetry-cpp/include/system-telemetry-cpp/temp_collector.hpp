#ifndef TEMP_COLLECTOR_HPP_
#define TEMP_COLLECTOR_HPP_

#include <fstream>

#include "telemetry_collector.hpp"

class TempCollector : public TelemetryCollector {
public:
  TempCollector();
  void collect(interfaces::msg::SystemTelemetry &msg) override;

private:
  float read_temp(std::ifstream &file);

  std::ifstream core_file_;
  std::ifstream case_file_;
};

#endif // TEMP_COLLECTOR_HPP_
