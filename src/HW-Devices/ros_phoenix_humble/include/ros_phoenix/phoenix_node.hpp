#ifndef ROS_PHOENIX_BASE_COMPONENT
#define ROS_PHOENIX_BASE_COMPONENT

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rcutils/logging_macros.h"
#include "ros_phoenix/base_node.hpp"

#define Phoenix_No_WPI  // remove WPI dependencies
#include <chrono>
#include <stdexcept>

#include "ctre/Phoenix.h"

using namespace rclcpp;
using namespace std::chrono_literals;

namespace ros_phoenix {

template <class MotorController, class Configuration, class FeedbackDevice,
          class ControlMode>
class PhoenixNode : public BaseNode {
 public:
  explicit PhoenixNode(const std::string &name,
                       const NodeOptions &options = NodeOptions())
      : BaseNode(name, options) {
    this->controller_ =
        std::make_shared<MotorController>(this->id_, this->interface_);
  }

  virtual ~PhoenixNode() {
    if (this->controller_) {
      this->controller_->Set(ControlMode::Disabled, 0.0);
    }
  }

  virtual MotorStatus::SharedPtr status() {
    MotorStatus::SharedPtr status =
        std::make_shared<ros_phoenix::msg::MotorStatus>();

    status->temperature = this->controller_->GetTemperature();
    status->bus_voltage = this->controller_->GetBusVoltage();

    status->output_percent = this->controller_->GetMotorOutputPercent();
    status->output_voltage = this->controller_->GetMotorOutputVoltage();
    status->output_current = this->get_output_current();

    status->position = this->controller_->GetSelectedSensorPosition() *
                           this->sensor_multiplier_ -
                       this->sensor_offset_;
    // CTRE library returns velocity in units/100ms. Multiply by 10 to get
    // units/s.
    status->velocity = this->controller_->GetSelectedSensorVelocity() * 10.0 *
                       this->sensor_multiplier_;

    return status;
  }

  virtual void set(MotorControl::SharedPtr control_msg) {
    if (this->follow_id_ >= 0) {
      return;
    }

    BaseNode::set(control_msg);

    ControlMode mode = static_cast<ControlMode>(control_msg->mode);
    if (mode == ControlMode::Velocity) {
      // CTRE library expects velocity in units/100ms
      this->controller_->Set(
          mode, control_msg->value / 10.0 / this->sensor_multiplier_);
    } else if (mode == ControlMode::Position) {
      this->controller_->Set(mode, (this->sensor_offset_ + control_msg->value) /
                                       this->sensor_multiplier_);
    } else if (mode == ControlMode::PercentOutput ||
               mode == ControlMode::Disabled) {
      this->controller_->Set(mode, control_msg->value);
    } else {
      this->controller_->Set(ControlMode::Disabled, 0.0);
      RCLCPP_WARN(this->get_logger(), "Invalid control mode: %d",
                  static_cast<int>(mode));
    }
  }

  virtual rcl_interfaces::msg::SetParametersResult reconfigure(
      const std::vector<rclcpp::Parameter> &params) {
    std::lock_guard<std::mutex> guard(this->config_mutex_);

    for (auto &param : params) {
      if (param.get_name() == Parameter::ID) {
        this->id_ = param.as_int();
        this->controller_ =
            std::make_shared<MotorController>(this->id_, this->interface_);
      } else if (param.get_name() == Parameter::INTERFACE) {
        this->interface_ = param.as_string();
        this->controller_ =
            std::make_shared<MotorController>(this->id_, this->interface_);
      }
    }

    return BaseNode::reconfigure(params);
  }

 protected:
  virtual void configure() {
    bool warned = false;
    while (!this->configured_) {
      std::this_thread::sleep_for(std::chrono::seconds(1));
      std::lock_guard<std::mutex> guard(this->config_mutex_);
      if (this->configured_) {
        break;  // Break out if signaled to stop while sleeping
      }

      if (this->controller_->GetFirmwareVersion() == -1) {
        if (!warned) {
          RCLCPP_WARN(
              this->get_logger(),
              "Motor controller has not been seen and cannot be configured!");
        }
        continue;
      }
      SlotConfiguration slot;
      slot.kP = this->get_parameter("P").as_double();
      slot.kI = this->get_parameter("I").as_double();
      slot.kD = this->get_parameter("D").as_double();
      slot.kF = this->get_parameter("F").as_double();

      Configuration config;
      config.slot0 = slot;
      config.voltageCompSaturation =
          this->get_parameter("max_voltage").as_double();
      config.pulseWidthPeriod_EdgesPerRot =
          this->get_parameter("edges_per_rot").as_int();
      this->configure_current_limit(config);

      ErrorCode error =
          this->controller_->ConfigAllSettings(config, 50);  // Takes up to 50ms
      if (error != ErrorCode::OK) {
        if (!warned) {
          RCLCPP_WARN(this->get_logger(),
                      "Motor controller configuration failed!");
          warned = true;
        }
        continue;
      }

      if (this->get_parameter("brake_mode").as_bool())
        this->controller_->SetNeutralMode(NeutralMode::Brake);
      else
        this->controller_->SetNeutralMode(NeutralMode::Coast);

      this->configure_sensor();
      this->controller_->ConfigSelectedFeedbackCoefficient(1.0);

      this->controller_->EnableVoltageCompensation(true);
      this->controller_->SetInverted(this->get_parameter("invert").as_bool());
      this->controller_->SetSensorPhase(
          this->get_parameter("invert_sensor").as_bool());
      this->controller_->SelectProfileSlot(0, 0);

      if (this->follow_id_ >= 0) {
        this->controller_->Set(ControlMode::Follower, this->follow_id_);
      }

      RCLCPP_INFO(this->get_logger(),
                  "Successfully configured Motor Controller");
      this->configured_ = true;
    }
  }

  // Methods to be reimplemented by specific motor controllers
  virtual void configure_current_limit(Configuration &config) = 0;
  virtual void configure_sensor() = 0;
  virtual double get_output_current() = 0;

  std::shared_ptr<MotorController> controller_;
};

}  // namespace ros_phoenix

#endif  // ROS_PHOENIX_BASE_COMPONENT
