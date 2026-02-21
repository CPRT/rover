#include "TalonSRXWrapper.hpp"

namespace motors = ctre::phoenix::motorcontrol;

TalonSRXWrapper::TalonSRXWrapper(const hardware_interface::ComponentInfo &joint,
                                 rclcpp::Node::SharedPtr debug_node)
    : BaseWrapper(joint, debug_node), id_(-1), kP_(0.0), kI_(0.0), kD_(0.0),
      kF_(0.0),
      control_type_(ctre::phoenix::motorcontrol::ControlMode::Disabled),
      sensor_type_(SensorType::RELATIVE), sensor_ticks_(4096),
      sensor_offset_ticks_(0), crossover_mode_(false), inverted_(false),
      invert_sensor_(false), debug_pub_(nullptr), talon_controller_(nullptr),
      initialized_(false) {
  std::string sensor_type_str;
  std::string can_interface = "can0";
  double sensor_offset = 0.0;
  for (const auto &param : joint.parameters) {
    if (param.first == "can_id") {
      id_ = std::stoi(param.second);
    } else if (param.first == "sensor_type") {
      sensor_type_str = param.second;
    } else if (param.first == "sensor_ticks") {
      sensor_ticks_ = std::stoi(param.second);
    } else if (param.first == "sensor_offset") {
      sensor_offset = std::stod(param.second);
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
  sensor_offset_ticks_ =
      static_cast<int>(sensor_offset * sensor_ticks_ / (2.0 * M_PI));
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
  if (!initialized_) {
    if ((debug_node_->now() - start_time_).seconds() > kWaitDurationSec) {
      initialized_ = true;
      read();
      if (control_type_ == motors::ControlMode::Position) {
        command_ = position_;
      } else {
        command_ = 0.0;
      }
      RCLCPP_INFO(debug_node_->get_logger(), "TalonSRX initialized for id %d",
                  id_);
    }
    return;
  }
  double output = 0.0;
  if (crossover_mode_ && std::abs(command_) > M_PI) {
    RCLCPP_WARN_THROTTLE(
        debug_node_->get_logger(), *debug_node_->get_clock(), 1000,
        "%s: Command %.2f is outside of [-pi, pi] in crossover mode, which may "
        "cause unexpected behavior for id %d",
        __FUNCTION__, command_, id_);
  }
  if (control_type_ == motors::ControlMode::Position) {
    if (crossover_mode_) {
      double normalized = (command_ + M_PI) / (2.0 * M_PI);
      double ticks = normalized * sensor_ticks_;
      double ticks_with_offset = ticks + sensor_offset_ticks_;
      output = fmod(ticks_with_offset, sensor_ticks_);
      if (output < 0)
        output += sensor_ticks_;
    } else {
      output = (command_)*sensor_ticks_ / (2.0 * M_PI) + sensor_offset_ticks_;
    }
  } else if (control_type_ == motors::ControlMode::Velocity) {
    // Talons use d / 100ms as vel
    output = (command_ * sensor_ticks_ / (2.0 * M_PI)) / 10.0;
  }
  talon_controller_->Set(control_type_, output);
}

void TalonSRXWrapper::read() {
  int raw_position =
      talon_controller_->GetSelectedSensorPosition() - sensor_offset_ticks_;
  if (crossover_mode_) {
    raw_position %= sensor_ticks_;
    if (raw_position < 0) {
      raw_position += sensor_ticks_;
    }
    double normalized = static_cast<double>(raw_position) / sensor_ticks_;
    position_ = normalized * 2.0 * M_PI - M_PI;
  } else {
    position_ = raw_position * (2.0 * M_PI) / sensor_ticks_;
  }
  double raw_velocity = talon_controller_->GetSelectedSensorVelocity();
  // Talons use d / 100ms as vel
  velocity_ = raw_velocity * 10 * (2.0 * M_PI) / sensor_ticks_;
}

void TalonSRXWrapper::configure() {
  BaseWrapper::configure();
  while (true) {
    if (talon_controller_->GetFirmwareVersion() == -1) {
      RCLCPP_ERROR_THROTTLE(
          debug_node_->get_logger(), *debug_node_->get_clock(), 500,
          "%s: Motor controller for id %d not responding, retrying...",
          __FUNCTION__, id_);
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
    config.peakCurrentDuration = 100;
    talon_controller_->EnableCurrentLimit(true);
    ErrorCode error = talon_controller_->ConfigAllSettings(config, 50);

    if (error != ErrorCode::OK) {
      RCLCPP_ERROR_THROTTLE(
          debug_node_->get_logger(), *debug_node_->get_clock(), 500,
          "%s: Failed to configure motor controller (%d) with code: %d",
          __FUNCTION__, id_, error);
      continue;
    }
    break;
  }
  if (sensor_type_ == SensorType::PWM) {
    talon_controller_->ConfigSelectedFeedbackSensor(
        TalonSRXFeedbackDevice::CTRE_MagEncoder_Absolute);
  } else if (sensor_type_ == SensorType::ANALOG) {
    talon_controller_->ConfigSelectedFeedbackSensor(
        TalonSRXFeedbackDevice::Analog);
  } else {
    talon_controller_->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder,
                                                    0, 0);
  }

  talon_controller_->SetNeutralMode(NeutralMode::Brake);
  talon_controller_->ConfigFeedbackNotContinuous(crossover_mode_);
  talon_controller_->ConfigSelectedFeedbackCoefficient(1.0);
  talon_controller_->EnableVoltageCompensation(true);
  talon_controller_->SetSensorPhase(invert_sensor_);
  talon_controller_->SetInverted(inverted_);
  talon_controller_->ConfigPulseWidthPeriod_FilterWindowSz(1);
  talon_controller_->SelectProfileSlot(0, 0);
  talon_controller_->SetStatusFramePeriod(
      StatusFrameEnhanced::Status_2_Feedback0, 20);
  talon_controller_->SetIntegralAccumulator(0);
  read();
  if (control_type_ == motors::ControlMode::Position) {
    command_ = position_;
  }
  talon_controller_->Set(motors::ControlMode::Disabled, 0.0);
  RCLCPP_INFO(debug_node_->get_logger(),
              "%s: Successfully configured Motor Controller %d", __FUNCTION__,
              id_);
  start_time_ = debug_node_->now();
}