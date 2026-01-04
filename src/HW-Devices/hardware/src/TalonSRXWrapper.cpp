#include "TalonSRXWrapper.hpp"

namespace motors = ctre::phoenix::motorcontrol;

TalonSRXWrapper::TalonSRXWrapper(const hardware_interface::ComponentInfo &joint,
                                 rclcpp::Node::SharedPtr debug_node)
    : BaseWrapper(joint, debug_node), id_(-1), kP_(0.0), kI_(0.0), kD_(0.0),
      kF_(0.0),
      control_type_(ctre::phoenix::motorcontrol::ControlMode::Disabled),
      sensor_type_(SensorType::RELATIVE), sensor_ticks_(4096),
      sensor_offset_(0.0), crossover_mode_(false), inverted_(false),
      invert_sensor_(false), debug_pub_(nullptr), talon_controller_(nullptr) {
  std::string sensor_type_str;
  std::string can_interface = "can0";

  for (const auto &param : joint.parameters) {
    if (param.first == "can_id") {
      id_ = std::stoi(param.second);
    } else if (param.first == "sensor_type") {
      sensor_type_str = param.second;
    } else if (param.first == "sensor_ticks") {
      sensor_ticks_ = std::stoi(param.second);
    } else if (param.first == "sensor_offset") {
      sensor_offset_ = std::stod(param.second);
    } else if (param.first == "crossover") {
      crossover_mode_ = (param.second == "true");
    } else if (param.first == "kP") {
      kP_ = std::stod(param.second);
    } else if (param.first == "kI") {
      kI_ = std::stod(param.second);
    } else if (param.first == "kD") {
      kD_ = std::stod(param.second);
    } else if (param.first == "kF") {
      kF_ = std::stod(param.second);
    } else if (param.first == "can_interface") {
      can_interface = param.second;
    } else if (param.first == "invert") {
      inverted_ = (param.second == "true");
    } else if (param.first == "invert_sensor") {
      invert_sensor_ = (param.second == "true");
    } else {
      RCLCPP_DEBUG(debug_node_->get_logger(),
                   "[%s] Unknown parameter: %s, ignoring", joint.name.c_str(),
                   param.first.c_str());
    }
  }
  if (id_ < 0) {
    RCLCPP_FATAL(debug_node_->get_logger(),
                 "[%s] Missing or invalid can_id parameter",
                 joint.name.c_str());
    throw std::runtime_error("Invalid can_id for TalonSRX");
  }

  if (joint.command_interfaces.size() != 1) {
    RCLCPP_FATAL(debug_node_->get_logger(),
                 "[%s] Expected exactly one command interface, found %zu",
                 joint.name.c_str(), joint.command_interfaces.size());
    throw std::runtime_error("Invalid command interfaces for TalonSRX");
  }

  if (joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION) {
    control_type_ = motors::ControlMode::Position;
  } else if (joint.command_interfaces[0].name ==
             hardware_interface::HW_IF_VELOCITY) {
    control_type_ = motors::ControlMode::Velocity;
  } else {
    RCLCPP_FATAL(debug_node_->get_logger(),
                 "[%s] Invalid command interface: %s, must be either POSITION "
                 "or VELOCITY",
                 joint.name.c_str(), joint.command_interfaces[0].name.c_str());
    throw std::runtime_error("Invalid command interface for TalonSRX");
  }

  talon_controller_ =
      std::make_shared<motors::can::TalonSRX>(id_, can_interface);
  debug_pub_ = debug_node_->create_publisher<ros_phoenix::msg::MotorStatus>(
      joint.name + "/status", rclcpp::SystemDefaultsQoS());

  std::transform(sensor_type_str.begin(), sensor_type_str.end(),
                 sensor_type_str.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  if (sensor_type_str == "quadrature") {
    sensor_type_ = SensorType::RELATIVE;
  } else if (sensor_type_str == "absolute") {
    sensor_type_ = SensorType::PWM;
  } else if (sensor_type_str == "analog") {
    sensor_type_ = SensorType::ANALOG;
  }
}

void TalonSRXWrapper::pub_status() const {
  if (!talon_controller_ || !debug_pub_) {
    return;
  }
  ros_phoenix::msg::MotorStatus status_msg;
  status_msg.temperature = talon_controller_->GetTemperature();
  status_msg.bus_voltage = talon_controller_->GetBusVoltage();
  status_msg.output_percent = talon_controller_->GetMotorOutputPercent();
  status_msg.output_voltage = talon_controller_->GetMotorOutputVoltage();
  status_msg.output_current = talon_controller_->GetOutputCurrent();
  status_msg.position = position_;
  status_msg.velocity = velocity_;
  status_msg.fwd_limit = talon_controller_->IsFwdLimitSwitchClosed() == 1;
  status_msg.rev_limit = talon_controller_->IsRevLimitSwitchClosed() == 1;
  debug_pub_->publish(status_msg);
}

void TalonSRXWrapper::write() {
  double output = 0.0;
  if (control_type_ == motors::ControlMode::Position) {
    output = command_ * sensor_ticks_ / (2.0 * M_PI);
  } else if (control_type_ == motors::ControlMode::Velocity) {
    output = command_ * sensor_ticks_ / (2.0 * M_PI) /
             10.0; // Convert rad/s to ticks per 100ms
  }
  talon_controller_->Set(control_type_, output);
}

int TalonSRXWrapper::wrap_symmetric(const int x, const int range) {
  assert(range > 0);

  const int half = range / 2;

  int y = (x + half) % range;
  if (y < 0)
    y += range;

  return y - half;
}

void TalonSRXWrapper::read() {
  int raw_ticks =
      static_cast<int>(talon_controller_->GetSelectedSensorPosition());
  if (crossover_mode_) {
    // adjust between -sensor_ticks_/2 and sensor_ticks_/2
    raw_ticks = wrap_symmetric(raw_ticks, sensor_ticks_);
  }
  position_ = raw_ticks * 2.0 * M_PI / sensor_ticks_;
  double raw_velocity = talon_controller_->GetSelectedSensorVelocity();
  // Convert raw velocity to rad/s
  // The TalonSRX reports velocity in ticks per 100ms, so we need to convert
  // it to radians per second.
  velocity_ = (raw_velocity / sensor_ticks_) * 2.0 * M_PI * 10;
}

void TalonSRXWrapper::configure() {
  BaseWrapper::configure();
  while (true) {
    if (talon_controller_->GetFirmwareVersion() == -1) {
      RCLCPP_ERROR_THROTTLE(
          debug_node_->get_logger(), *debug_node_->get_clock(), 500,
          "%s: Motor controller not responding, retrying...", __FUNCTION__);
      continue;
    }
    SlotConfiguration slot;
    slot.kP = kP_;
    slot.kI = kI_;
    slot.kD = kD_;
    slot.kF = kF_;

    TalonSRXConfiguration config;
    config.slot0 = slot;
    config.voltageCompSaturation = 24.0;
    config.pulseWidthPeriod_EdgesPerRot = sensor_ticks_;
    config.continuousCurrentLimit = 10.0;
    config.peakCurrentLimit = 10.0;
    config.peakCurrentDuration = 100; // ms
    talon_controller_->EnableCurrentLimit(true);
    ErrorCode error =
        talon_controller_->ConfigAllSettings(config, /*timeout*/ 50);
    if (error != ErrorCode::OK) {
      RCLCPP_ERROR_THROTTLE(
          debug_node_->get_logger(), *debug_node_->get_clock(), 500,
          "%s: Failed to configure motor controller with code: %d",
          __FUNCTION__, error);
      continue;
    }
    break;
  }

  int current_ticks = 0;
  int offset_ticks =
      static_cast<int>(sensor_offset_ * sensor_ticks_ / (2.0 * M_PI));
  if (sensor_type_ == SensorType::PWM) {
    auto &sc = talon_controller_->GetSensorCollection();
    current_ticks = sc.GetPulseWidthPosition() & 0xFFF;
  } else if (sensor_type_ == SensorType::ANALOG) {
    auto &sc = talon_controller_->GetSensorCollection();
    current_ticks = sc.GetAnalogIn() & 0xFFF;
  }
  if (invert_sensor_) {
    current_ticks = sensor_ticks_ - current_ticks;
  }
  if (crossover_mode_) {
    current_ticks = wrap_symmetric(current_ticks, sensor_ticks_);
  }
  int real_ticks = current_ticks - offset_ticks;

  // Talon SRX expects a double for SetSelectedSensorPosition
  double ticks_double = static_cast<double>(real_ticks);
  // If exactly one of invert_sensor_ or inverted_ is true, negate the ticks.
  // This ensures that if both are true (double inversion), the effect cancels
  // out.
  bool final_invert = invert_sensor_ != inverted_;
  if (final_invert) {
    ticks_double = -ticks_double;
  }

  talon_controller_->SetNeutralMode(NeutralMode::Brake);
  talon_controller_->ConfigSelectedFeedbackCoefficient(1.0);

  // The Talons virtualize a QuadEncoder from the PWM signal if no quad encoder
  // is present. The absolute part is set at the beginning from the
  // SetSelectedSensorPosition
  talon_controller_->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,
                                                  0, 0);
  talon_controller_->EnableVoltageCompensation(true);
  talon_controller_->SetInverted(inverted_);
  talon_controller_->SetSensorPhase(invert_sensor_);
  talon_controller_->SelectProfileSlot(0, 0);
  talon_controller_->SetSelectedSensorPosition(ticks_double, 0, 50);
  RCLCPP_INFO(debug_node_->get_logger(),
              "%s: Successfully configured Motor Controller %d, with offset %d",
              __FUNCTION__, id_, real_ticks);
}