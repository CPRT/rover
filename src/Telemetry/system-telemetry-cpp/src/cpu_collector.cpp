#include "cpu_collector.hpp"

#include <sstream>
#include <string>

CPUCollector::CPUCollector()
    : stat_file_("/proc/stat"), prev_stats_(read_cpu_stats()) {}

CPUCollector::CPUStats CPUCollector::read_cpu_stats() {
  CPUStats stats{0, 0};
  stat_file_.clear();
  stat_file_.seekg(0, std::ios::beg);
  std::string line;
  if (std::getline(stat_file_, line)) {
    std::istringstream iss(line);
    std::string cpu;
    iss >> cpu;
    uint64_t user, nice, system, idle, iowait, irq, softirq, steal;
    iss >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal;
    stats.idle = idle + iowait;
    stats.total = user + nice + system + idle + iowait + irq + softirq + steal;
  } else {
    RCLCPP_ERROR(logger_, "Failed to read /proc/stat");
  }
  return stats;
}

double CPUCollector::calculate_cpu_usage(const CPUStats& prev,
                                         const CPUStats& curr) {
  const uint64_t total_diff = curr.total - prev.total;
  const uint64_t idle_diff = curr.idle - prev.idle;
  if (total_diff == 0) {
    return 0.0;
  }
  return 100.0 * (total_diff - idle_diff) / static_cast<double>(total_diff);
}

void CPUCollector::collect(interfaces::msg::SystemTelemetry& msg) {
  CPUStats current = read_cpu_stats();
  double usage = calculate_cpu_usage(prev_stats_, current);
  prev_stats_ = current;
  msg.cpu_usage = static_cast<float>(usage);
}
