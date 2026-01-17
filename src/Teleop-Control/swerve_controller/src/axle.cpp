#include "swerve_controller/axle.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <math.h>

bool Axle::activate(
    std::vector<hardware_interface::LoanedCommandInterface> &command_interfaces,
    std::vector<hardware_interface::LoanedStateInterface> &state_interfaces) {
  const auto command_handle = std::find_if(
      command_interfaces.begin(), command_interfaces.end(),
      [this](const auto &interface) {
        return interface.get_prefix_name() == steering_angle_name_ &&
               interface.get_interface_name() ==
                   hardware_interface::HW_IF_POSITION;
      });
  if (command_handle == command_interfaces.end()) {
    return false;
  }
  steering_angle_ = *command_handle;
  const auto wheel_command_handle = std::find_if(
      command_interfaces.begin(), command_interfaces.end(),
      [this](const auto &interface) {
        return interface.get_prefix_name() == wheel_velocity_name_ &&
               interface.get_interface_name() ==
                   hardware_interface::HW_IF_VELOCITY;
      });
  if (wheel_command_handle == command_interfaces.end()) {
    return false;
  }
  wheel_velocity_ = *wheel_command_handle;
  const auto state_handle = std::find_if(
      state_interfaces.begin(), state_interfaces.end(),
      [this](const auto &interface) {
        return interface.get_prefix_name() == steering_angle_name_ &&
               interface.get_interface_name() ==
                   hardware_interface::HW_IF_POSITION;
      });
  if (state_handle != state_interfaces.end()) {
    steering_angle_state_ = *state_handle;
  }
  const auto wheel_state_handle = std::find_if(
      state_interfaces.begin(), state_interfaces.end(),
      [this](const auto &interface) {
        return interface.get_prefix_name() == wheel_velocity_name_ &&
               interface.get_interface_name() ==
                   hardware_interface::HW_IF_VELOCITY;
      });
  if (wheel_state_handle != state_interfaces.end()) {
    wheel_velocity_state_ = *wheel_state_handle;
  }

  return true;
}

static inline double wrap_pi(double a) {
  // (-pi, pi]
  return std::remainder(a, 2.0 * M_PI);
}

static inline double shortest_angular_distance(double from, double to) {
  return wrap_pi(to - from);
}

// If turning more than 90deg, flip wheel direction and add pi to steering.
static inline void optimize_steering(double current_angle, double &target_angle,
                                     double &target_wheel_rad_s) {
  double d = shortest_angular_distance(current_angle, target_angle);
  if (std::abs(d) > (M_PI * 0.5)) {
    target_angle = wrap_pi(target_angle + M_PI);
    target_wheel_rad_s = -target_wheel_rad_s;
  }
}

bool Axle::update(double steering_angle_cmd, double wheel_velocity_cmd) {
  bool ok = true;

  if (steering_angle_) {
    steering_angle_->get().set_value(steering_angle_cmd);
  } else {
    ok = false;
  }

  if (wheel_velocity_) {
    wheel_velocity_->get().set_value(wheel_velocity_cmd);
  } else {
    ok = false;
  }

  return ok;
}

bool Axle::stop() {
  if (!wheel_velocity_) {
    return false;
  }
  wheel_velocity_->get().set_value(0.0);
  return true;
}

std::optional<double> Axle::currentAngle() const {
  if (steering_angle_state_) {
    return steering_angle_state_->get().get_value();
  }
  return std::nullopt;
}

std::optional<double> Axle::currentVelocity() const {
  if (wheel_velocity_state_) {
    return wheel_velocity_state_->get().get_value();
  }
  return std::nullopt;
}
