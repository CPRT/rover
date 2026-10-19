#ifndef CDH_LINK_HPP_
#define CDH_LINK_HPP_

#include "rclcpp/rclcpp.hpp"

#include "interfaces/msg/system_telemetry.hpp"

namespace cdh_link {

class CdhLink : public rclcpp::Node {
public:
  explicit CdhLink();

private:
  void
  system_telemetry_cb(const interfaces::msg::SystemTelemetry::SharedPtr msg);

  rclcpp::Subscription<interfaces::msg::SystemTelemetry>::SharedPtr
      system_telemetry_sub_;
};

} // namespace cdh_link

#endif // CDH_LINK_HPP_