#include "science_node.hpp"

#include <fcntl.h>
#include <sys/stat.h>

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
  this->declare_parameter<uint16_t>("sweep_timeout", 60);
  this->declare_parameter<std::string>("output_dir", "/usr/local/zed/polar/");

  mkdir(this->get_parameter("output_dir").as_string().c_str(), 0777);

  this->adc_pub_ =
      this->create_publisher<interfaces::msg::ScienceADC>("/science/adc", 10);
  this->temp_pub_ =
      this->create_publisher<interfaces::msg::DHT22>("/science/temp", 10);
  this->co2_pub_ =
      this->create_publisher<std_msgs::msg::UInt16>("/science/co2", 10);
  this->polar_pub_ = this->create_publisher<interfaces::msg::PolarimeterSweep>(
      "/science/polarimeter", 10);

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

  this->polar_srv_ = this->create_service<interfaces::srv::RunPolarimeter>(
      "/science/run_polarimeter",
      std::bind(&ScienceNode::polar_callback, this, std::placeholders::_1,
                std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(), "Science Sensors Node started");
}

void ScienceNode::deinit() {
  motor_evt_.deinit();
  servo_evt_.deinit();
  req_polar_evt_.deinit();
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
  if (!req_polar_evt_.init(event_loop,
                           std::bind(&ScienceNode::request_polar, this))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Failed to initialize polarimeter request event");
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
  case CmdId::PolarData: {
    if (!verify_length("PolarData", 7, frame.can_dlc))
      break;
    std::lock_guard<std::mutex> guard(polar_mutex_);
    if (frame.data[0] != polar_index_)
      RCLCPP_WARN(this->get_logger(),
                  "Polarimeter indexing mismatch: expected packet starting at "
                  "index %d, got %d",
                  polar_index_, frame.data[0]);
    polar_index_ = frame.data[0];
    polar_points_[polar_index_++] = (frame.data[1] << 8) | frame.data[2];
    polar_points_[polar_index_++] = (frame.data[3] << 8) | frame.data[4];
    polar_points_[polar_index_++] = (frame.data[5] << 8) | frame.data[6];

    if (polar_index_ == SCAN_STEPS) {
      polar_cond_.notify_one();
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

void ScienceNode::polar_callback(
    const std::shared_ptr<interfaces::srv::RunPolarimeter::Request> request,
    std::shared_ptr<interfaces::srv::RunPolarimeter::Response> response) {
  {
    std::unique_lock<std::mutex> guard(polar_mutex_);
    for (int i = 0; i < SCAN_STEPS; i++) {
      polar_points_[i] = 0;
    }
    polar_index_ = 0;
  }
  req_polar_evt_.set();

  std::unique_lock<std::mutex> guard(polar_mutex_);
  if (polar_cond_.wait_for(
          guard, std::chrono::seconds(
                     this->get_parameter("sweep_timeout").as_int())) ==
      std::cv_status::timeout) {
    response->success = false;
    response->message = "Timeout waiting for polarimeter samples";
    return;
  } else {
    if (polar_index_ != SCAN_STEPS) {
      RCLCPP_WARN(this->get_logger(),
                  "Polar tried writing without a full buffer");
      response->success = false;
      response->message = "Tried writing without a full sample buffer";
      return;
    }
    if (this->polar_pub_) {
      interfaces::msg::PolarimeterSweep msg;
      msg.readings.assign(polar_points_, polar_points_ + SCAN_STEPS);
      this->polar_pub_->publish(msg);
    }

    char path[128];
    snprintf(path, sizeof(path), "%spolarimeter_%s_%.0f.csv",
             this->get_parameter("output_dir").as_string().c_str(),
             request->title.c_str(), this->now().seconds());
    int fd = open(path, O_CREAT | O_WRONLY, 0777);

    char contents[1024];
    uint16_t pos = 0;

    pos += snprintf(contents, sizeof(contents) - pos, "step, reading\n");

    for (int i = 0; i < SCAN_STEPS; i++) {
      pos += snprintf(contents + pos, sizeof(contents) - pos, "%d, %d\n", i,
                      polar_points_[i]);
    }
    write(fd, contents, pos - 1);
    close(fd);
    response->success = true;
    response->message = "Polarimeter samples saved";
    response->file_path = path;
    RCLCPP_INFO(this->get_logger(), "Polarimeter sweep saved to %s", path);
  }
}

void ScienceNode::request_polar() {
  struct can_frame frame;
  frame.can_id = node_id_ << 5 | CmdId::PolarScan;
  frame.can_dlc = 0;
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