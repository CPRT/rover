#include "TalonDriveController.hpp"

#include <algorithm>
#include <cmath>

TalonDriveController::TalonDriveController(const rclcpp::NodeOptions &options)
    : rclcpp::Node("talon_drive_controller", options) {
  lastTimestamp_ = 0;
  this->declare_parameter("max_speed", 2.0);
  maxSpeed_ = this->get_parameter("max_speed").as_double();
  this->declare_parameter("base_width", 0.9144);
  baseWidth_ = this->get_parameter("base_width").as_double();
  this->declare_parameter("pub_odom", true);
  pubOdom_ = this->get_parameter("pub_odom").as_bool();
  this->declare_parameter("pub_elec", true);
  pubElec_ = this->get_parameter("pub_elec").as_bool();
  this->declare_parameter("angular_cov", 0.3);
  angularCov_ = this->get_parameter("angular_cov").as_double();
  this->declare_parameter("linear_cov", 0.3);
  linearCov_ = this->get_parameter("linear_cov").as_double();
  this->declare_parameter("wheel_rad", 0.2);
  wheelCircumference_ = this->get_parameter("wheel_rad").as_double() * 2 * M_PI;
  this->declare_parameter("frame_id", "base_link");
  frameId_ = this->get_parameter("frame_id").as_string();
  this->declare_parameter("frequency", 10.0);
  const double frequency =
      std::max(this->get_parameter("frequency").as_double(), 1.0);
  this->declare_parameter("timeout", 2.0);
  timeout_ = this->get_parameter("timeout").as_double();
  this->declare_parameter("angular_slip_ratio", 1.0);
  angularSlipRatio_ = this->get_parameter("angular_slip_ratio").as_double();
  if (angularSlipRatio_ <= 0.0 || angularSlipRatio_ > 1.0) {
    RCLCPP_WARN(
        this->get_logger(),
        "Angular slip ratio should be in range (0.0, 1.0], setting to 1.0");
    angularSlipRatio_ = 1.0;
  }
  this->declare_parameter("low_latency_mode", true);
  low_latency_mode_ = this->get_parameter("low_latency_mode").as_bool();

  this->declare_parameter(
      "wheels", std::vector<std::string>{"frontRight", "frontLeft", "backRight",
                                         "backLeft"});
  std::vector<std::string> wheel_names =
      this->get_parameter("wheels").as_string_array();

  for (const auto &wheel_name : wheel_names) {
    wheels_.emplace_back(WheelControl(wheel_name, this));
    if (pubOdom_) {
      statusSubs_.emplace_back(this->create_subscription<MotorStatus>(
          wheel_name + "/status", 10,
          std::bind(&WheelControl::setStatus, &wheels_.back(),
                    std::placeholders::_1)));
    }
  }

  RCLCPP_INFO(this->get_logger(), "There are %ld wheels in the drive",
              wheels_.size());

  twistSub_ = this->create_subscription<Twist>(
      "/cmd_vel", 10,
      std::bind(&TalonDriveController::twist_callback, this,
                std::placeholders::_1));
  const int periodMs = 1000 / frequency;
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(periodMs),
      std::bind(&TalonDriveController::control_timer_callback, this));

  if (pubOdom_) {
    odomPub_ = this->create_publisher<Odometry>("/drive/odom", 10);
    odomTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(periodMs),
        std::bind(&TalonDriveController::odom_pub_callback, this));
  }
}

void TalonDriveController::odom_pub_callback() {
  double rotationsLeft = 0.0;
  double rotationsRight = 0.0;

  for (const auto &wheel : wheels_) {
    if (wheel.getWheelSide() == WheelSide::LEFT) {
      rotationsLeft += wheel.getVelocity();
    } else {
      rotationsRight -= wheel.getVelocity();
    }
  }
  const int wheelsPerSide = wheels_.size() / 2;
  const double vl = rotationsLeft * wheelCircumference_ / wheelsPerSide;
  const double vr = rotationsRight * wheelCircumference_ / wheelsPerSide;

  Odometry odom;
  odom.header.stamp = this->get_clock()->now();
  odom.header.frame_id = "odom";
  odom.child_frame_id = frameId_;
  odom.twist.twist.linear.x = (vr + vl) / 2;
  odom.twist.twist.angular.z = angularSlipRatio_ * (vl - vr) / baseWidth_;
  odom.twist.covariance = {0};
  odom.twist.covariance[0] = linearCov_;
  odom.twist.covariance[35] = angularCov_;
  odomPub_->publish(odom);
}

void TalonDriveController::control_timer_callback() {
  if (this->get_clock()->now().seconds() - lastTimestamp_ > timeout_) {
    for (auto &wheel : wheels_) {
      wheel.setVelocity(0.0);
    }
  }
  for (const auto &wheel : wheels_) {
    wheel.send();
  }
}

void TalonDriveController::twist_callback(const Twist::SharedPtr msg) {
  lastTimestamp_ = this->get_clock()->now().seconds();

  double linearX = msg->linear.x;
  if (linearX > maxSpeed_) {
    linearX = maxSpeed_;
  }
  if (linearX < -maxSpeed_) {
    linearX = -maxSpeed_;
  }
  const double angular_portion =
      msg->angular.z * baseWidth_ / angularSlipRatio_;

  double vr = linearX + angular_portion / 2;
  double vl = linearX - angular_portion / 2;

  for (auto &wheel : wheels_) {
    if (wheel.getWheelSide() == WheelSide::LEFT) {
      wheel.setVelocity(vl / wheelCircumference_);
    } else {
      wheel.setVelocity(-vr / wheelCircumference_);
    }
    if (low_latency_mode_) {
      wheel.send();
    }
  }
}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(TalonDriveController)