
#include "can_helpers.hpp"
#include "can_simple_messages.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "odrive_enums.h"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_status.hpp"
#include "socket_can.hpp"
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace odrive_ros2_control {

class Axis;
class GravityFFManager;

class ODriveHardwareInterface final
    : public hardware_interface::SystemInterface {
public:
  using return_type = hardware_interface::return_type;
  using State = rclcpp_lifecycle::State;

  CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;
  CallbackReturn on_configure(const State &previous_state) override;
  CallbackReturn on_cleanup(const State &previous_state) override;
  CallbackReturn on_activate(const State &previous_state) override;
  CallbackReturn on_deactivate(const State &previous_state) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  return_type perform_command_mode_switch(
      const std::vector<std::string> &start_interfaces,
      const std::vector<std::string> &stop_interfaces) override;

  return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  void on_can_msg(const can_frame &frame);
  void set_axis_command_mode(const Axis &axis);
  void pub_status();

  bool active_;
  EpollEventLoop event_loop_;
  std::vector<Axis> axes_;
  std::string can_intf_name_;
  SocketCanIntf can_intf_;
  rclcpp::Time timestamp_;
  rclcpp::Node::SharedPtr debug_node_;
  rclcpp::TimerBase::SharedPtr debug_timer_;
  int debug_frequency_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread spin_thread_;
};

struct Axis {
  Axis(SocketCanIntf *can_intf, uint32_t node_id, double multiplier,
       ODriveInputMode input_mode,
       std::shared_ptr<GravityFFManager> gravity_ff_manager = nullptr)
      : can_intf_(can_intf), node_id_(node_id), multiplier_(multiplier),
        input_mode_(input_mode),
        gravity_ff_manager_(std::move(gravity_ff_manager)) {}

  void on_can_msg(const rclcpp::Time &timestamp, const can_frame &frame);

  void on_can_msg();

  SocketCanIntf *can_intf_;
  uint32_t node_id_;
  double multiplier_;

  // Commands (ros2_control => ODrives)
  double pos_setpoint_ = 0.0f;    // [rad]
  double vel_setpoint_ = 0.0f;    // [rad/s]
  double torque_setpoint_ = 0.0f; // [Nm]

  // State (ODrives => ros2_control)
  // rclcpp::Time encoder_estimates_timestamp_;
  // uint32_t axis_error_ = 0;
  // uint8_t axis_state_ = 0;
  // uint8_t procedure_result_ = 0;
  // uint8_t trajectory_done_flag_ = 0;
  double pos_estimate_ = NAN; // [rad]
  double vel_estimate_ = NAN; // [rad/s]
  // double iq_setpoint_ = NAN;
  // double iq_measured_ = NAN;
  double torque_target_ = NAN;   // [Nm]
  double torque_estimate_ = NAN; // [Nm]
  uint32_t active_errors_ = 0;
  uint32_t disarm_reason_ = 0;
  // double fet_temperature_ = NAN;
  // double motor_temperature_ = NAN;
  double bus_voltage_ = NAN;
  double bus_current_ = NAN;

  // Indicates which controller inputs are enabled. This is configured by the
  // controller that sits on top of this hardware interface. Multiple inputs
  // can be enabled at the same time, in this case the non-primary inputs are
  // used as feedforward terms.
  // This implicitly defines the ODrive's control mode.
  bool pos_input_enabled_ = false;
  bool vel_input_enabled_ = false;
  bool torque_input_enabled_ = false;

  ODriveInputMode input_mode_;

  rclcpp::Publisher<ros_phoenix::msg::MotorStatus>::SharedPtr debug_pub_;
  std::shared_ptr<GravityFFManager> gravity_ff_manager_;

  template <typename T> bool send(const T &msg) const {
    struct can_frame frame;
    frame.can_id = node_id_ << 5 | msg.cmd_id;
    frame.can_dlc = msg.msg_length;
    msg.encode_buf(frame.data);
    if (!can_intf_->send_can_frame(frame)) {
      RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"),
                  "Could not send CAN msg");
      return false;
    }
    return true;
  }
};

class GravityFFManager {
public:
  GravityFFManager(rclcpp::Node::SharedPtr node, double gravity_ff_freq,
                   std::string link_frame, std::string gravity_frame,
                   double gravity_const)
      : node_(node), gravity_ff_freq_(gravity_ff_freq),
        gravity_frame_(gravity_frame), link_frame_(link_frame),
        gravity_const_(gravity_const) {

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }
  double get_ff_value() { return gravity_ff_.load(std::memory_order_relaxed); }
  void start() {
    if (gravity_ff_freq_ <= 0 || gravity_frame_.empty() ||
        link_frame_.empty()) {
      return;
    }
    gravity_ff_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / gravity_ff_freq_)),
        std::bind(&GravityFFManager::update_gravity_ff, this));
  }

private:
  void update_gravity_ff() {
    if (!tf_buffer_) {
      RCLCPP_ERROR(node_->get_logger(),
                   "%s: TF buffer not initialized, cannot update gravity FF",
                   __FUNCTION__);
      return;
    }
    try {
      auto tf = tf_buffer_->lookupTransform(
          gravity_frame_.c_str(), link_frame_.c_str(), tf2::TimePointZero);

      tf2::Quaternion q;
      tf2::fromMsg(tf.transform.rotation, q);

      double roll, pitch, yaw;
      tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
      double ff = gravity_const_ * std::sin(pitch);
      gravity_ff_.store(ff, std::memory_order_relaxed);
      RCLCPP_DEBUG(
          node_->get_logger(),
          "%s: Updated gravity FF to %.2f based on pitch %.2f degrees ",
          __FUNCTION__, ff, pitch * 180.0 / M_PI);
    } catch (const tf2::TransformException &err) {
      RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 500,
                            "%s: Caught exception %s", __FUNCTION__,
                            err.what());
    }
  }
  rclcpp::Node::SharedPtr node_;
  double gravity_ff_freq_;
  std::string gravity_frame_;
  std::string link_frame_;
  double gravity_const_;
  std::atomic<double> gravity_ff_{0.0};
  rclcpp::TimerBase::SharedPtr gravity_ff_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
};

} // namespace odrive_ros2_control

using namespace odrive_ros2_control;

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

CallbackReturn
ODriveHardwareInterface::on_init(const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  debug_node_ = std::make_shared<rclcpp::Node>("odrive_system_debug_node");
  executor_ = rclcpp::executors::SingleThreadedExecutor::make_shared();
  executor_->add_node(debug_node_);
  spin_thread_ = std::thread([this]() { this->executor_->spin(); });

  can_intf_name_ = info_.hardware_parameters["can"];
  debug_frequency_ = 0;
  if (info_.hardware_parameters.find("debug_freq") !=
      info_.hardware_parameters.end()) {
    debug_frequency_ = std::stoi(info_.hardware_parameters.at("debug_freq"));
  }

  for (auto &joint : info_.joints) {
    if (joint.parameters.find("node_id") == joint.parameters.end()) {
      RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"),
                   "Joint %s is missing required parameter 'node_id'",
                   joint.name.c_str());
      return CallbackReturn::ERROR;
    }
    double multiplier = 1.0;
    if (joint.parameters.find("multiplier") != joint.parameters.end()) {
      multiplier = std::stod(joint.parameters.at("multiplier"));
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                  "Setting Joint %s multiplier to %f'", joint.name.c_str(),
                  multiplier);
    }
    ODriveInputMode input_mode = INPUT_MODE_PASSTHROUGH;
    if (joint.parameters.find("input_mode") != joint.parameters.end()) {
      const std::string input_mode_str = joint.parameters.at("input_mode");
      if (input_mode_str == "Traj") {
        input_mode = INPUT_MODE_TRAP_TRAJ;
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                    "Setting Joint %s input type to Trajectory",
                    joint.name.c_str());
      } else if (input_mode_str == "Passthrough") {
        input_mode = INPUT_MODE_PASSTHROUGH;
        RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                    "Setting Joint %s input type to Passthrough",
                    joint.name.c_str());
      } else {
        RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"),
                    "Unknown input type %s on Joint %s", input_mode_str.c_str(),
                    joint.name.c_str());
      }
    }
    double gravity_ff_freq = 0.0;
    if (joint.parameters.find("gravity_ff_freq") != joint.parameters.end()) {
      gravity_ff_freq = std::stod(joint.parameters.at("gravity_ff_freq"));
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                  "Setting Joint %s gravity_ff_freq to %f'", joint.name.c_str(),
                  gravity_ff_freq);
    }
    double gravity_const = 0.0;
    if (joint.parameters.find("gravity_const") != joint.parameters.end()) {
      gravity_const = std::stod(joint.parameters.at("gravity_const"));
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                  "Setting Joint %s gravity_const to %f'", joint.name.c_str(),
                  gravity_const);
    }
    std::string gravity_frame = "base_link";
    if (joint.parameters.find("gravity_frame") != joint.parameters.end()) {
      gravity_frame = std::string(joint.parameters.at("gravity_frame"));
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                  "Setting Joint %s gravity_frame to %s'", joint.name.c_str(),
                  gravity_frame.c_str());
    }
    std::string link_frame;
    if (joint.parameters.find("current_frame") != joint.parameters.end()) {
      link_frame = std::string(joint.parameters.at("current_frame"));
      RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                  "Setting Joint %s current_frame to %s'", joint.name.c_str(),
                  link_frame.c_str());
    }
    std::shared_ptr<GravityFFManager> gravity_ff_manager = nullptr;
    if (gravity_ff_freq > 0.0 && !gravity_frame.empty() &&
        !link_frame.empty()) {
      gravity_ff_manager = std::make_shared<GravityFFManager>(
          debug_node_, gravity_ff_freq, link_frame, gravity_frame,
          gravity_const);
    }
    axes_.emplace_back(&can_intf_, std::stoi(joint.parameters.at("node_id")),
                       multiplier, input_mode, gravity_ff_manager);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_configure(const State &) {
  if (!can_intf_.init(
          can_intf_name_, &event_loop_,
          std::bind(&ODriveHardwareInterface::on_can_msg, this, _1))) {
    RCLCPP_ERROR(rclcpp::get_logger("ODriveHardwareInterface"),
                 "Failed to initialize SocketCAN on %s",
                 can_intf_name_.c_str());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
              "Initialized SocketCAN on %s", can_intf_name_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_cleanup(const State &) {
  can_intf_.deinit();
  return CallbackReturn::SUCCESS;
}

CallbackReturn ODriveHardwareInterface::on_activate(const State &) {
  RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
              "activating ODrives...");

  // This can be called several seconds before the controller finishes starting.
  // Therefore we enable the ODrives only in perform_command_mode_switch().

  active_ = true;
  for (auto &axis : axes_) {
    set_axis_command_mode(axis);
  }
  if (debug_frequency_ > 0) {
    debug_timer_ = debug_node_->create_wall_timer(
        std::chrono::milliseconds(1000 / debug_frequency_),
        std::bind(&ODriveHardwareInterface::pub_status, this));
    for (auto &axis : axes_) {
      axis.debug_pub_ =
          debug_node_->template create_publisher<ros_phoenix::msg::MotorStatus>(
              info_.joints[&axis - &axes_[0]].name + "/status",
              rclcpp::SystemDefaultsQoS());
      if (axis.gravity_ff_manager_) {
        axis.gravity_ff_manager_->start();
      }
    }
  }

  return CallbackReturn::SUCCESS;
}

void ODriveHardwareInterface::pub_status() {
  for (const auto &axis : axes_) {
    ros_phoenix::msg::MotorStatus status_msg;
    status_msg.bus_voltage = axis.bus_voltage_;
    status_msg.output_current = axis.bus_current_;
    status_msg.position = axis.pos_estimate_;
    status_msg.velocity = axis.vel_estimate_;
    status_msg.active_errors = axis.active_errors_;
    axis.debug_pub_->publish(status_msg);
  }
}

CallbackReturn ODriveHardwareInterface::on_deactivate(const State &) {
  RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
              "deactivating ODrives...");

  active_ = false;
  for (auto &axis : axes_) {
    set_axis_command_mode(axis);
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ODriveHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &axes_[i].torque_target_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &axes_[i].vel_estimate_));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &axes_[i].pos_estimate_));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
ODriveHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &axes_[i].torque_setpoint_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &axes_[i].vel_setpoint_));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &axes_[i].pos_setpoint_));
  }

  return command_interfaces;
}

return_type ODriveHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string> &start_interfaces,
    const std::vector<std::string> &stop_interfaces) {
  for (size_t i = 0; i < axes_.size(); ++i) {
    Axis &axis = axes_[i];
    std::array<std::pair<std::string, bool *>, 3> interfaces = {
        {{info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION,
          &axis.pos_input_enabled_},
         {info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY,
          &axis.vel_input_enabled_},
         {info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT,
          &axis.torque_input_enabled_}}};

    bool mode_switch = false;

    for (const std::string &key : stop_interfaces) {
      for (auto &kv : interfaces) {
        if (kv.first == key) {
          *kv.second = false;
          mode_switch = true;
        }
      }
    }

    for (const std::string &key : start_interfaces) {
      for (auto &kv : interfaces) {
        if (kv.first == key) {
          *kv.second = true;
          mode_switch = true;
        }
      }
    }

    if (mode_switch) {
      set_axis_command_mode(axis);
    }
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::read(const rclcpp::Time &timestamp,
                                          const rclcpp::Duration &) {
  timestamp_ = timestamp;

  while (can_intf_.read_nonblocking()) {
    // repeat until CAN interface has no more messages
  }

  return return_type::OK;
}

return_type ODriveHardwareInterface::write(const rclcpp::Time &,
                                           const rclcpp::Duration &) {
  for (auto &axis : axes_) {
    // Send the CAN message that fits the set of enabled setpoints
    if (axis.pos_input_enabled_) {
      auto target = axis.pos_setpoint_;
      if (std::isnan(axis.pos_setpoint_)) {
        target = axis.pos_estimate_;
      }
      Set_Input_Pos_msg_t msg;
      msg.Input_Pos = target / (2 * M_PI * axis.multiplier_);
      msg.Vel_FF = axis.vel_input_enabled_
                       ? (axis.vel_setpoint_ / (2 * M_PI * axis.multiplier_))
                       : 0.0f;
      if (axis.torque_input_enabled_) {
        msg.Torque_FF = axis.torque_setpoint_;
      } else if (axis.gravity_ff_manager_) {
        msg.Torque_FF = axis.gravity_ff_manager_->get_ff_value();
      } else {
        msg.Torque_FF = 0.0f;
      }

      axis.send(msg);
    } else if (axis.vel_input_enabled_) {
      Set_Input_Vel_msg_t msg;
      msg.Input_Vel = axis.vel_setpoint_ / (2 * M_PI * axis.multiplier_);
      msg.Input_Torque_FF =
          axis.torque_input_enabled_ ? axis.torque_setpoint_ : 0.0f;
      axis.send(msg);
    } else if (axis.torque_input_enabled_) {
      Set_Input_Torque_msg_t msg;
      msg.Input_Torque = axis.torque_setpoint_;
      axis.send(msg);
    } else {
      // no control enabled - don't send any setpoint
    }
  }

  return return_type::OK;
}

void ODriveHardwareInterface::on_can_msg(const can_frame &frame) {
  for (auto &axis : axes_) {
    if ((frame.can_id >> 5) == axis.node_id_) {
      axis.on_can_msg(timestamp_, frame);
    }
  }
}

void ODriveHardwareInterface::set_axis_command_mode(const Axis &axis) {
  if (!active_) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                "Interface inactive. Setting axis to idle.");
    Set_Axis_State_msg_t idle_msg;
    idle_msg.Axis_Requested_State = AXIS_STATE_IDLE;
    axis.send(idle_msg);
    return;
  }

  Set_Controller_Mode_msg_t control_msg;
  Clear_Errors_msg_t clear_error_msg;
  Set_Axis_State_msg_t state_msg;

  clear_error_msg.Identify = 0;
  control_msg.Input_Mode = INPUT_MODE_PASSTHROUGH;
  state_msg.Axis_Requested_State = AXIS_STATE_CLOSED_LOOP_CONTROL;

  if (axis.pos_input_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                "Setting to position control.");
    control_msg.Control_Mode = CONTROL_MODE_POSITION_CONTROL;
    control_msg.Input_Mode =
        INPUT_MODE_TRAP_TRAJ; // CPRT HACK: This should be configurable
  } else if (axis.vel_input_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                "Setting to velocity control.");
    control_msg.Control_Mode = CONTROL_MODE_VELOCITY_CONTROL;
  } else if (axis.torque_input_enabled_) {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                "Setting to torque control.");
    control_msg.Control_Mode = CONTROL_MODE_TORQUE_CONTROL;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("ODriveHardwareInterface"),
                "No control mode specified. Setting to idle.");
    state_msg.Axis_Requested_State = AXIS_STATE_IDLE;
    axis.send(state_msg);
    return;
  }

  axis.send(control_msg);
  axis.send(clear_error_msg);
  axis.send(state_msg);
}

void Axis::on_can_msg(const rclcpp::Time &, const can_frame &frame) {
  uint8_t cmd = frame.can_id & 0x1f;

  auto try_decode = [&]<typename TMsg>(TMsg &msg) {
    if (frame.can_dlc < Get_Encoder_Estimates_msg_t::msg_length) {
      RCLCPP_WARN(rclcpp::get_logger("ODriveHardwareInterface"),
                  "message %d too short", cmd);
      return false;
    }
    msg.decode_buf(frame.data);
    return true;
  };

  switch (cmd) {
  case Get_Encoder_Estimates_msg_t::cmd_id: {
    if (Get_Encoder_Estimates_msg_t msg; try_decode(msg)) {
      pos_estimate_ = msg.Pos_Estimate * (2 * M_PI * multiplier_);
      vel_estimate_ = msg.Vel_Estimate * (2 * M_PI * multiplier_);
    }
  } break;
  case Get_Torques_msg_t::cmd_id: {
    if (Get_Torques_msg_t msg; try_decode(msg)) {
      torque_target_ = msg.Torque_Target;
      torque_estimate_ = msg.Torque_Estimate;
    }
  } break;
  case Get_Bus_Voltage_Current_msg_t::cmd_id: {
    if (Get_Bus_Voltage_Current_msg_t msg; try_decode(msg)) {
      bus_voltage_ = msg.Bus_Voltage;
      bus_current_ = msg.Bus_Current;
    }
  } break;
  case Get_Error_msg_t::cmd_id: {
    if (Get_Error_msg_t msg; try_decode(msg)) {
      active_errors_ = msg.Active_Errors;
      disarm_reason_ = msg.Disarm_Reason;
    }
  } break;
    // silently ignore unimplemented command IDs
  }
}

PLUGINLIB_EXPORT_CLASS(odrive_ros2_control::ODriveHardwareInterface,
                       hardware_interface::SystemInterface)
