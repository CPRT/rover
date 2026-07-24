#include "science_node.hpp"

enum CmdId : uint32_t {
  SetMotor = 0x00,
  SetServo = 0x01,
  PolarScan = 0x02,
  ADCData = 0x03,
  TempData = 0x04,
  CO2Data = 0x05,
  PolarData = 0x06
};

ScienceNode::ScienceNode(const std::string name,
                         const rclcpp::NodeOptions &options)
    : rclcpp::Node(name, options) {
  this->declare_parameter<std::string>("interface", "can0");
  this->declare_parameter<uint16_t>("node_id", 30);

  this->adc_pub_ =
      this->create_publisher<interfaces::msg::ScienceADC>("/science/adc", 10);
  this->temp_pub_ =
      this->create_publisher<interfaces::msg::DHT22>("/science/temp", 10);
  this->co2_pub_ =
      this->create_publisher<std_msgs::msg::UInt16>("/science/co2", 10);

  this->motor_sub_ = this->create_subscription<interfaces::msg::ScienceMotor>(
      "/science/motor", rclcpp::QoS(1).reliable(),
      [this](const interfaces::msg::ScienceMotor::SharedPtr msg) {
        std::lock_guard<std::mutex> guard(motor_mutex_);
        motor_msg_ = *msg;
        motor_evt_.set();
      });

  this->servo_sub_ = this->create_subscription<interfaces::msg::ScienceServo>(
      "/science/servo", rclcpp::QoS(1).reliable(),
      [this](const interfaces::msg::ScienceServo::SharedPtr msg) {
        std::lock_guard<std::mutex> guard(servo_mutex_);
        servo_msg_ = *msg;
        servo_evt_.set();
      });

  RCLCPP_INFO(this->get_logger(), "Science Sensors Node started");
}

void ScienceNode::deinit() {
  //   if (axis_idle_on_shutdown_) {
  //     struct can_frame frame;
  //     frame.can_id = node_id_ << 5 | CmdId::kSetAxisState;
  //     write_le<uint32_t>(ODriveAxisState::AXIS_STATE_IDLE, frame.data);
  //     frame.can_dlc = 4;
  //     can_intf_.send_can_frame(frame);
  //   }

  //   sub_evt_.deinit();
  //   srv_evt_.deinit();
  can_intf_.deinit();
}

bool ScienceNode::init(EpollEventLoop *event_loop) {
  node_id_ = this->get_parameter("node_id").as_int();
  std::string interface = this->get_parameter("interface").as_string();

  if (!can_intf_.init(interface, event_loop,
                      std::bind(&ScienceNode::recv_callback, this,
                                std::placeholders::_1))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize socket can interface: %s",
                 interface.c_str());
    return false;
  }
  if (!motor_evt_.init(event_loop,
                       std::bind(&ScienceNode::motor_callback, this))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize motor subscriber event");
    return false;
  }
  if (!servo_evt_.init(event_loop,
                       std::bind(&ScienceNode::servo_callback, this))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize servo subscriber event");
    return false;
  }
  RCLCPP_INFO(this->get_logger(), "node_id: %d", node_id_);
  RCLCPP_INFO(this->get_logger(), "interface: %s", interface.c_str());
  return true;
}

void ScienceNode::recv_callback(const can_frame &frame) {
  if (((frame.can_id >> 5) & 0x3F) != node_id_)
    return;

  switch (frame.can_id & 0x1F) {
  case CmdId::ADCData: {
    if (!verify_length("ADCData", 6, frame.can_dlc))
      break;
    interfaces::msg::ScienceADC msg;
    msg.adc1 = (frame.data[0] << 8) | frame.data[1];
    msg.adc2 = (frame.data[2] << 8) | frame.data[3];
    msg.adc3 = (frame.data[4] << 8) | frame.data[5];
    if (adc_pub_) {
      adc_pub_->publish(msg);
    }
    break;
  }
  case CmdId::TempData: {
    if (!verify_length("TempData", 8, frame.can_dlc))
      break;
    interfaces::msg::DHT22 msg;
    uint32_t temp_bytes = (frame.data[0] << 24) | (frame.data[1] << 16) |
                          (frame.data[2] << 8) | frame.data[3];
    uint32_t humid_bytes = (frame.data[4] << 24) | (frame.data[5] << 16) |
                           (frame.data[6] << 8) | frame.data[7];
    memcpy(&msg.temperature, &temp_bytes, sizeof(temp_bytes));
    memcpy(&msg.humidity, &humid_bytes, sizeof(humid_bytes));
    if (temp_pub_) {
      temp_pub_->publish(msg);
    }
    break;
  }
  case CmdId::CO2Data: {
    if (!verify_length("CO2Data", 2, frame.can_dlc))
      break;
    std_msgs::msg::UInt16 msg;
    msg.data = (frame.data[0] << 8) | frame.data[1];
    if (co2_pub_) {
      co2_pub_->publish(msg);
    }
    break;
  }
  case CmdId::SetMotor:
  case CmdId::SetServo:
  case CmdId::PolarScan: {
    break; // Ignore commands coming from another master/host on the bus
  }
  default: {
    RCLCPP_WARN(this->get_logger(), "Received unused message: ID = 0x%x",
                (frame.can_id & 0x1F));
    break;
  }
  }
}

void ScienceNode::motor_callback() {
  struct can_frame frame;
  frame.can_id = node_id_ << 5 | CmdId::SetMotor;
  {
    std::lock_guard<std::mutex> guard(motor_mutex_);
    frame.data[0] = motor_msg_.pin;
    frame.data[1] = motor_msg_.duty_cycle;
    frame.data[2] = (motor_msg_.duration >> 8) & 0xff;
    frame.data[3] = motor_msg_.duration & 0xff;
    frame.data[4] = (motor_msg_.ramp >> 8) & 0xff;
    frame.data[5] = motor_msg_.ramp & 0xff;
  }
  frame.can_dlc = 6;
  can_intf_.send_can_frame(frame);
}

void ScienceNode::servo_callback() {
  struct can_frame frame;
  frame.can_id = node_id_ << 5 | CmdId::SetServo;
  {
    std::lock_guard<std::mutex> guard(servo_mutex_);
    frame.data[0] = servo_msg_.pin;
    frame.data[1] = (servo_msg_.us >> 8) & 0xff;
    frame.data[2] = servo_msg_.us & 0xff;
  }
  frame.can_dlc = 3;
  can_intf_.send_can_frame(frame);
}

inline bool ScienceNode::verify_length(const std::string &name,
                                       uint8_t expected, uint8_t length) {
  bool valid = expected == length;
  RCLCPP_DEBUG(this->get_logger(), "received %s", name.c_str());
  if (!valid)
    RCLCPP_WARN(this->get_logger(), "Incorrect %s frame length: %d != %d",
                name.c_str(), length, expected);
  return valid;
}