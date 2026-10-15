#include "temp_collector.hpp"

#include <sstream>
#include <string>

TempCollector::TempCollector()
    : core_file_("/sys/class/hwmon/hwmon8/temp1_input"),
      case_file_("/sys/class/hwmon/hwmon1/temp1_input") {}

float TempCollector::read_temp(std::ifstream &file) {
  float temp = 0.f;
  file.clear();
  file.seekg(0, std::ios::beg);
  std::string line;
  if (std::getline(file, line)) {
    std::istringstream iss(line);
    uint64_t temp_mdeg;
    iss >> temp_mdeg;
    temp = (float)temp_mdeg / 1000;
  } else {
    RCLCPP_ERROR(logger_, "Failed to read temperature file");
  }
  return temp;
}

void TempCollector::collect(interfaces::msg::SystemTelemetry &msg) {
  msg.core_temp = read_temp(core_file_);
  msg.case_temp = read_temp(case_file_);
}
