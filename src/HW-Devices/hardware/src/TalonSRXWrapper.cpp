#include "TalonSRXWrapper.hpp"
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

namespace motors = ctre::phoenix::motorcontrol;

TalonSRXWrapper::TalonSRXWrapper(const hardware_interface::ComponentInfo &joint,
                                 rclcpp::Node::SharedPtr debug_node)
    : BaseWrapper(joint, debug_node), id_(-1), kP_(0.0), kI_(0.0), kD_(0.0),
      kF_(0.0),
      control_type_(ctre::phoenix::motorcontrol::ControlMode::Disabled),
      sensor_type_(SensorType::RELATIVE), sensor_ticks_(4096),
      sensor_offset_ticks_(0), crossover_mode_(false), inverted_(false),
      invert_sensor_(false), debug_pub_(nullptr), gravity_const_(0.0),
      talon_controller_(nullptr), initialized_(false), gravity_ff_(0),
      gravity_ff_freq_(0) {
  std::string sensor_type_str;
  std::string can_interface = "can0";
  double sensor_offset = 0.0;
  target_frame_ = "base_link";
  cur_frame_ = "";
  for (const auto &param : joint.parameters) {
    if (param.first == "can_id") {
      id_ = std::stoi(param.second);
    } else if (param.first == "sensor_type") {
      sensor_type_ = sensor_type_from_str(param.second);
    } else if (param.first == "load_sensor") {
      load_sensor_ = sensor_type_from_str(param.second);
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
    } else if (param.first == "gravity_ff_freq") {
      gravity_ff_freq_ = std::stoi(param.second);
    } else if (param.first == "gravity_frame") {
      target_frame_ = param.second;
    } else if (param.first == "current_frame") {
      cur_frame_ = param.second;
    } else if (param.first == "gravity_const") {
      gravity_const_ = std::stod(param.second);
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
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(debug_node_->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  talon_controller_ =
      std::make_shared<motors::can::TalonSRX>(id_, can_interface);
  debug_pub_ = debug_node_->create_publisher<ros_phoenix::msg::MotorStatus>(
      joint.name + "/status", rclcpp::SystemDefaultsQoS());

  sensor_offset_ticks_ =
      static_cast<int>(sensor_offset * sensor_ticks_ / (2.0 * M_PI));
}

TalonSRXWrapper::SensorType
TalonSRXWrapper::sensor_type_from_str(std::string str) {
  std::transform(str.begin(), str.end(), str.begin(),
                 [](unsigned char c) { return std::tolower(c); });
  const auto itr = sensor_type_map.find(str);
  if (itr == sensor_type_map.end()) {
    return SensorType::NONE;
  }
  return itr->second;
}

void TalonSRXWrapper::update_gravity_ff() {
  if (!tf_buffer_) {
    RCLCPP_ERROR(debug_node_->get_logger(),
                 "%s: TF buffer not initialized, cannot update gravity FF",
                 __FUNCTION__);
    return;
  }
  try {
    auto tf = tf_buffer_->lookupTransform(
        target_frame_.c_str(), cur_frame_.c_str(), tf2::TimePointZero);

    tf2::Quaternion q;
    tf2::fromMsg(tf.transform.rotation, q);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double ff = gravity_const_ * std::sin(pitch);
    gravity_ff_.store(ff, std::memory_order_relaxed);
    RCLCPP_DEBUG(
        debug_node_->get_logger(),
        "%s: Updated gravity FF to %.2f based on pitch %.2f degrees for id %d",
        __FUNCTION__, ff, pitch * 180.0 / M_PI, id_);
  } catch (const tf2::TransformException &err) {
    RCLCPP_ERROR_THROTTLE(debug_node_->get_logger(), *debug_node_->get_clock(),
                          500, "%s: Caught exception %s", __FUNCTION__,
                          err.what());
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
  if (std::abs(command_ - position_) > M_PI &&
      control_type_ == motors::ControlMode::Position) {
    RCLCPP_WARN_THROTTLE(
        debug_node_->get_logger(), *debug_node_->get_clock(), 1000,
        "%s: Large position error (%.2f) for id %d, which may indicate a "
        "problem with sensor configuration or an unexpected jump in position",
        __FUNCTION__, command_ - position_, id_);
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
      output = command_ * sensor_ticks_ / (2.0 * M_PI) + sensor_offset_ticks_;
    }
  } else if (control_type_ == motors::ControlMode::Velocity) {
    // Talons use d / 100ms as vel
    output = (command_ * sensor_ticks_ / (2.0 * M_PI)) / 10.0;
  }
  talon_controller_->Set(control_type_, output,
                         motors::DemandType::DemandType_ArbitraryFeedForward,
                         gravity_ff_.load(std::memory_order_relaxed));
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

int TalonSRXWrapper::get_load_enc() const {
  auto &sensor_collection = talon_controller_->GetSensorCollection();
  int abs_ticks = 0;
  switch (load_sensor_) {
  case SensorType::PWM:
    return sensor_collection.GetPulseWidthPosition();
  case SensorType::ANALOG:
    return sensor_collection.GetAnalogIn();
  case SensorType::RELATIVE:
    return sensor_collection.GetQuadraturePosition();
  default:
    return 0;
  }
}

void TalonSRXWrapper::configure() {
  BaseWrapper::configure();
  // Start gravity ff timer
  if (gravity_ff_freq_ > 0 && !target_frame_.empty() && !cur_frame_.empty()) {
    gravity_ff_timer_ = debug_node_->create_wall_timer(
        std::chrono::milliseconds(1000 / gravity_ff_freq_),
        std::bind(&TalonSRXWrapper::update_gravity_ff, this));
    RCLCPP_INFO(
        debug_node_->get_logger(),
        "%s: Gravity FF enabled with frequency %d Hz, target frame '%s', "
        "current frame '%s', and gravity constant %.2f",
        __FUNCTION__, gravity_ff_freq_, target_frame_.c_str(),
        cur_frame_.c_str(), gravity_const_);
  } else {
    RCLCPP_INFO(
        debug_node_->get_logger(),
        "%s: Gravity FF disabled (frequency %d Hz, target frame '%s', current "
        "frame '%s')",
        __FUNCTION__, gravity_ff_freq_, target_frame_.c_str(),
        cur_frame_.c_str());
  }

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
  if (load_sensor_ != SensorType::NONE) {
    talon_controller_->SetSelectedSensorPosition(get_load_enc());
  }
  read();
  if (control_type_ == motors::ControlMode::Position) {
    command_ = position_;
  }
  talon_controller_->Set(motors::ControlMode::Disabled, 0.0);
  talon_controller_->ClearStickyFaults();
  RCLCPP_INFO(debug_node_->get_logger(),
              "%s: Successfully configured Motor Controller %d", __FUNCTION__,
              id_);
  start_time_ = debug_node_->now();
}