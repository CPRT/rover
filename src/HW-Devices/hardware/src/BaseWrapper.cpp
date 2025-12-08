#include "BaseWrapper.hpp"
BaseWrapper::BaseWrapper(const hardware_interface::ComponentInfo &joint,
                         std::shared_ptr<rclcpp::Node> debug_node)
    : info_(joint), debug_node_(std::move(debug_node)), position_(0.0),
      debug_timer_(nullptr), velocity_(0.0), command_(0.0), freq_(0) {
  for (const auto &param : joint.parameters) {
    if (param.first == "debug_frequency") {
      freq_ = std::stoi(param.second);
    }
  }
}

void BaseWrapper::configure() {
  if (freq_ > 0) {
    debug_timer_ = debug_node_->create_wall_timer(
        std::chrono::milliseconds(1000 / freq_),
        std::bind(&BaseWrapper::pub_status, this));
  }
}