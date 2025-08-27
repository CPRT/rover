#include "ros_phoenix/base_node.hpp"

#include "ros_phoenix/phoenix_manager.hpp"

namespace ros_phoenix {

const std::string BaseNode::Parameter::ID = "id";
const std::string BaseNode::Parameter::INTERFACE = "interface";

BaseNode::BaseNode(const std::string &name, const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options) {
  // Detect if component run outside of a phoenix container
  if (!PhoenixManager::instanceCreated()) {
    RCLCPP_FATAL(this->get_logger(),
                 "Phoenix components must be run inside phoenix container!");
    throw new std::runtime_error(
        "Phoenix components must be run inside phoenix container!");
  }

  this->declare_parameter<int>(Parameter::ID, 0);
  this->declare_parameter<std::string>(Parameter::INTERFACE, "can0");
  this->declare_parameter<int>("watchdog_ms", 100);
  this->template declare_parameter<int>("period_ms", 20);
  this->declare_parameter<int>("follow_id", -1);
  this->declare_parameter<int>("edges_per_rot",
                               2048);  // Encoder edges per rotation (2048 is
                                       // for Falcon built-in encoder)
  this->declare_parameter<bool>("invert", false);
  this->declare_parameter<bool>("invert_sensor", false);
  this->declare_parameter<bool>("non_continuous", false);
  this->declare_parameter<bool>("brake_mode", true);
  this->declare_parameter<int>("input_type", RELATIVE);
  this->declare_parameter<double>("max_voltage", 12);
  this->declare_parameter<double>("max_current", 30);
  this->declare_parameter<double>("sensor_multiplier", 1.0);
  this->declare_parameter<double>("sensor_offset", 0.0);

  this->declare_parameter<double>("P", 0);
  this->declare_parameter<double>("I", 0);
  this->declare_parameter<double>("D", 0);
  this->declare_parameter<double>("F", 0);

  this->id_ = this->get_parameter(Parameter::ID).as_int();
  this->interface_ = this->get_parameter(Parameter::INTERFACE).as_string();
  this->watchdog_ms_ = this->get_parameter("watchdog_ms").as_int();
  this->period_ms_ = this->get_parameter("period_ms").as_int();
  this->follow_id_ = this->get_parameter("follow_id").as_int();
  this->sensor_multiplier_ =
      this->get_parameter("sensor_multiplier").as_double();
  this->sensor_offset_ = this->get_parameter("sensor_offset").as_double();

  this->last_update_ = this->now();

  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(this->get_parameter("period_ms").as_int()),
      std::bind(&BaseNode::onTimer, this));

  this->set_parameters_callback_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> &params) {
        return this->reconfigure(params);
      });
  this->reconfigure({});
}

BaseNode::~BaseNode() {
  std::unique_lock<std::mutex> lock(this->config_mutex_);
  if (this->config_thread_) {
    this->configured_ = true;  // Signal config thread to stop
    lock.unlock();
    this->config_thread_->join();
  }
}

void BaseNode::set(MotorControl::SharedPtr control_msg
                   __attribute__((unused))) {
  if (!this->configured_) {
    control_msg = std::make_shared<MotorControl>();
    control_msg->mode = MotorControl::DISABLED;
    control_msg->value = 0.0;
  }

  if (this->follow_id_ < 0 && control_msg->mode != MotorControl::DISABLED) {
    this->last_update_ = this->now();
    if (this->watchdog_warned_) {
      RCLCPP_INFO(this->get_logger(),
                  "Motor output enabled after receiving command");
    }
    this->watchdog_warned_ = false;
  }
}

rcl_interfaces::msg::SetParametersResult BaseNode::reconfigure(
    const std::vector<rclcpp::Parameter> &params) {
  for (auto param : params) {
    RCLCPP_DEBUG(this->get_logger(), "Parameter changed: %s=%s",
                 param.get_name().c_str(), param.value_to_string().c_str());
    if (param.get_name() == "watchdog_ms") {
      this->watchdog_ms_ = param.as_int();

    } else if (param.get_name() == "period_ms") {
      this->timer_ =
          this->create_wall_timer(std::chrono::milliseconds(param.as_int()),
                                  std::bind(&BaseNode::onTimer, this));

    } else if (param.get_name() == "follow_id") {
      this->follow_id_ = param.as_int();

    } else if (param.get_name() == "sensor_multiplier") {
      this->sensor_multiplier_ = param.as_double();
    } else if (param.get_name() == "sensor_offset") {
      this->sensor_offset_ = param.as_double();
    }
  }

  if (!this->config_thread_) {  // Thread does not exist, create it
    this->configured_ = false;
    this->config_thread_ =
        std::make_shared<std::thread>(std::bind(&BaseNode::configure, this));
  } else if (this->configured_) {  // Thread needs to be joined and restarted
    this->config_thread_->join();
    this->configured_ = false;
    this->config_thread_ =
        std::make_shared<std::thread>(std::bind(&BaseNode::configure, this));
  }

  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  return result;
}

void BaseNode::onTimer() {
  if (!this->configured_) return;

  // Check if watchdog has expired
  if (this->follow_id_ < 0 &&
      this->now() - this->last_update_ >
          std::chrono::milliseconds(this->watchdog_ms_)) {
    MotorControl::SharedPtr msg = std::make_shared<MotorControl>();
    msg->mode = MotorControl::DISABLED;
    msg->value = 0.0;
    this->set(msg);

    if (!this->watchdog_warned_) {
      RCLCPP_WARN(this->get_logger(),
                  "Watchdog timer expired: Motor ouput disabled");
      this->watchdog_warned_ = true;
    }
  }
}

}  // namespace ros_phoenix
