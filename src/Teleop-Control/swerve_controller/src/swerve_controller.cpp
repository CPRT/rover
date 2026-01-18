#include "swerve_controller/swerve_controller.hpp"

#include <algorithm>
#include <cmath>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

namespace swerve_controller {

static inline double wrap_pi(double a) {
  // (-pi, pi]
  return std::remainder(a, 2.0 * M_PI);
}

static inline double shortest_angular_distance(double from, double to) {
  return wrap_pi(to - from);
}

double SwerveController::isValidAngle(double angle) {
  return angle > min_angle_ && angle < max_angle_;
}

// If turning more than 90deg, flip wheel direction and add pi to steering.
void SwerveController::optimize_steering(double current_angle,
                                         double &target_angle,
                                         double &target_wheel_rad_s) {
  double option1_angle = target_angle;
  double option2_angle = wrap_pi(target_angle + M_PI);
  if (isValidAngle(option1_angle) && isValidAngle(option2_angle)) {
    double diff1 =
        std::abs(shortest_angular_distance(current_angle, option1_angle));
    double diff2 =
        std::abs(shortest_angular_distance(current_angle, option2_angle));
    if (diff2 < diff1) {
      target_angle = option2_angle;
      target_wheel_rad_s = -target_wheel_rad_s;
    }
  } else if (isValidAngle(option1_angle)) {
    target_angle = option1_angle;
  } else if (isValidAngle(option2_angle)) {
    target_angle = option2_angle;
    target_wheel_rad_s = -target_wheel_rad_s;
  } else {
    RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "SwerveController: Both steering options invalid (angle=%.3f)",
        target_angle);
    target_angle = current_angle;
    target_wheel_rad_s = 0.0;
  }
}

controller_interface::InterfaceConfiguration
SwerveController::command_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &axle : axles_) {
    conf_names.push_back(axle.steering_angle_name() + "/" +
                         hardware_interface::HW_IF_POSITION);
    conf_names.push_back(axle.wheel_velocity_name() + "/" +
                         hardware_interface::HW_IF_VELOCITY);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          conf_names};
}

controller_interface::InterfaceConfiguration
SwerveController::state_interface_configuration() const {
  std::vector<std::string> conf_names;
  for (const auto &axle : axles_) {
    conf_names.push_back(axle.steering_angle_name() + "/" +
                         hardware_interface::HW_IF_POSITION);
    conf_names.push_back(axle.wheel_velocity_name() + "/" +
                         hardware_interface::HW_IF_VELOCITY);
  }
  return {controller_interface::interface_configuration_type::INDIVIDUAL,
          conf_names};
}

controller_interface::CallbackReturn SwerveController::on_init() {
  auto node = get_node();

  RCLCPP_INFO(node->get_logger(), "SwerveController initializing.");

  // ---- Command handling ----
  auto_declare<bool>("use_stamped_cmds", false);
  auto_declare<double>("cmd_timeout",
                       0.25); // seconds (0 = disabled)

  // ---- Robot geometry ----
  auto_declare<double>("wheel_base", 0.6);   // meters
  auto_declare<double>("track_width", 0.5);  // meters
  auto_declare<double>("wheel_radius", 0.1); // meters
  auto_declare<double>("min_angle", -1.57);
  auto_declare<double>("max_angle", 1.57);

  // ---- Frames ----
  auto_declare<std::string>("odom_frame_id", "odom");
  auto_declare<std::string>("base_frame_id", "base_link");

  // ---- Odometry ----
  auto_declare<double>("odom_publish_rate", 50.0);
  auto_declare<std::vector<double>>("covariance", std::vector<double>(6, 0.1));

  // ---- Axle joint names----
  auto_declare<std::string>("axles.front_left.steering_angle_name",
                            "front_left_steering_joint");
  auto_declare<std::string>("axles.front_left.wheel_velocity_name",
                            "front_left_wheel_joint");

  auto_declare<std::string>("axles.front_right.steering_angle_name",
                            "front_right_steering_joint");
  auto_declare<std::string>("axles.front_right.wheel_velocity_name",
                            "front_right_wheel_joint");

  auto_declare<std::string>("axles.rear_left.steering_angle_name",
                            "rear_left_steering_joint");
  auto_declare<std::string>("axles.rear_left.wheel_velocity_name",
                            "rear_left_wheel_joint");

  auto_declare<std::string>("axles.rear_right.steering_angle_name",
                            "rear_right_steering_joint");
  auto_declare<std::string>("axles.rear_right.wheel_velocity_name",
                            "rear_right_wheel_joint");

  // ---- Read parameters ----
  node->get_parameter("wheel_base", wheel_base_);
  node->get_parameter("track_width", track_width_);
  node->get_parameter("wheel_radius", wheel_radius_);
  node->get_parameter("min_angle", min_angle_);
  node->get_parameter("max_angle", max_angle_);
  node->get_parameter("cmd_timeout", cmd_timeout_);

  node->get_parameter("odom_frame_id", odom_frame_id_);
  node->get_parameter("base_frame_id", base_frame_id_);
  node->get_parameter("odom_publish_rate", odom_publish_rate_);
  node->get_parameter("covariance", covariance_);

  // ---- Axle construction ----
  axles_.emplace_back(
      node->get_parameter("axles.front_left.steering_angle_name").as_string(),
      node->get_parameter("axles.front_left.wheel_velocity_name").as_string());

  axles_.emplace_back(
      node->get_parameter("axles.front_right.steering_angle_name").as_string(),
      node->get_parameter("axles.front_right.wheel_velocity_name").as_string());

  axles_.emplace_back(
      node->get_parameter("axles.rear_left.steering_angle_name").as_string(),
      node->get_parameter("axles.rear_left.wheel_velocity_name").as_string());

  axles_.emplace_back(
      node->get_parameter("axles.rear_right.steering_angle_name").as_string(),
      node->get_parameter("axles.rear_right.wheel_velocity_name").as_string());

  RCLCPP_INFO(node->get_logger(),
              "SwerveController initialized (L=%.3f, W=%.3f, r=%.3f)",
              wheel_base_, track_width_, wheel_radius_);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SwerveController::on_configure(const rclcpp_lifecycle::State &) {
  auto node = get_node();
  bool use_stamped_cmds = false;
  node->get_parameter("use_stamped_cmds", use_stamped_cmds);
  if (use_stamped_cmds) {
    RCLCPP_INFO(node->get_logger(),
                "SwerveController configured to use stamped cmd_vel.");
    stamped_subscription_ =
        node->create_subscription<geometry_msgs::msg::TwistStamped>(
            "~/cmd_vel_stamped", rclcpp::SystemDefaultsQoS(),
            std::bind(&SwerveController::on_stamped_message, this,
                      std::placeholders::_1));
  } else {
    RCLCPP_INFO(node->get_logger(),
                "SwerveController configured to use non-stamped cmd_vel.");
    subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
        "~/cmd_vel", rclcpp::SystemDefaultsQoS(),
        std::bind(&SwerveController::on_message, this, std::placeholders::_1));
  }

  node->get_parameter("odom_publish_rate", odom_publish_rate_);

  odometry_publisher_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
      "~/wheel_odom", rclcpp::SystemDefaultsQoS());
  realtime_odometry_publisher_ = std::make_shared<
      realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>(
      odometry_publisher_);

  publish_period_ = rclcpp::Duration::from_seconds(1.0 / odom_publish_rate_);

  next_publish_timestamp_ = node->now() + publish_period_;

  const double x_f = 0.5 * wheel_base_;
  const double x_r = -0.5 * wheel_base_;
  const double y_l = 0.5 * track_width_;
  const double y_r = -0.5 * track_width_;

  px_ = {x_f, x_f, x_r, x_r};
  py_ = {y_l, y_r, y_l, y_r};

  RCLCPP_INFO(node->get_logger(), "SwerveController configured.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SwerveController::on_activate(const rclcpp_lifecycle::State &) {
  auto node = get_node();
  if (command_interfaces_.size() < 8 || state_interfaces_.size() < 8) {
    RCLCPP_ERROR(node->get_logger(),
                 "Expected at least 8 command interfaces (got %zu) and at "
                 "least 8 state interfaces (got %zu)",
                 command_interfaces_.size(), state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  for (auto &axle : axles_) {
    if (!axle.activate(command_interfaces_, state_interfaces_)) {
      RCLCPP_ERROR(node->get_logger(),
                   "Failed to activate axle with given interfaces.");
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  for (auto &axle : axles_) {
    axle.update(0.0, 0.0);
  }

  RCLCPP_INFO(node->get_logger(), "SwerveController activated.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SwerveController::on_deactivate(const rclcpp_lifecycle::State &) {
  // Stop the robot
  for (auto &axle : axles_) {
    axle.update(0.0, 0.0);
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SwerveController::on_cleanup(const rclcpp_lifecycle::State &) {
  subscription_.reset();
  stamped_subscription_.reset();
  realtime_odometry_publisher_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
SwerveController::on_error(const rclcpp_lifecycle::State &) {
  RCLCPP_ERROR(get_node()->get_logger(),
               "SwerveController entered error state.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
SwerveController::update(const rclcpp::Time &time,
                         const rclcpp::Duration &period) {
  geometry_msgs::msg::TwistStamped cmd;
  received_velocity_msg_.get(cmd);

  if (cmd_timeout_ > 0.0) {
    const double age = (time - cmd.header.stamp).seconds();
    if (age > cmd_timeout_) {
      for (auto &axle : axles_) {
        axle.stop();
      }
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000,
                           "SwerveController: Command timeout (age=%.3f > "
                           "%.3f), stopping robot.",
                           age, cmd_timeout_);
    }
  }

  const double vx = cmd.twist.linear.x;
  const double vy = cmd.twist.linear.y;
  const double wz = cmd.twist.angular.z;

  for (size_t i = 0; i < axles_.size() && i < 4; ++i) {
    const double vix = vx - wz * py_[i];
    const double viy = vy + wz * px_[i];

    double target_angle = std::atan2(viy, vix);
    const double speed_m_s = std::hypot(vix, viy);
    double target_wheel = speed_m_s / wheel_radius_;

    const auto current_angle = axles_[i].currentAngle();
    if (!current_angle.has_value()) {
      RCLCPP_WARN_THROTTLE(
          get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "SwerveController: Unable to read current angle of axle %zu", i);
      continue;
    }
    optimize_steering(current_angle.value(), target_angle, target_wheel);
    // cosine scaling
    double angle_diff = std::abs(
        shortest_angular_distance(current_angle.value(), target_angle));
    double speed_scaling = std::cos(angle_diff);
    target_wheel *= speed_scaling;
    axles_[i].update(target_angle, target_wheel);
  }

  update_odometry_and_publish_(time);

  return controller_interface::return_type::OK;
}

bool SwerveController::read_axle_vectors(
    std::array<double, kNumModules> &x_components,
    std::array<double, kNumModules> &y_components) {
  for (size_t i = 0; i < kNumModules; ++i) {
    const auto angle_opt = axles_[i].currentAngle();
    if (!angle_opt.has_value()) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000,
                           "SwerveController: Unable to read current angle of "
                           "axle %zu for odometry",
                           i);
      return false;
    }

    const auto vel_opt = axles_[i].currentVelocity();
    if (!vel_opt.has_value()) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(),
                           1000,
                           "SwerveController: Unable to read current velocity "
                           "of axle %zu for odometry",
                           i);
      return false;
    }

    const double theta = angle_opt.value();
    const double speed = vel_opt.value() * wheel_radius_; // m/s

    x_components[i] = speed * std::cos(theta);
    y_components[i] = speed * std::sin(theta);
  }
  return true;
}

void SwerveController::build_covariance_matrix(
    std::array<double, 36> &cov_matrix, const std::vector<double> &cov_vector) {
  std::fill(cov_matrix.begin(), cov_matrix.end(), 0.0);
  if (cov_vector.size() == 6) {
    cov_matrix[0] = cov_vector[0];
    cov_matrix[7] = cov_vector[1];
    cov_matrix[14] = cov_vector[2];
    cov_matrix[21] = cov_vector[3];
    cov_matrix[28] = cov_vector[4];
    cov_matrix[35] = cov_vector[5];
  }
}

bool SwerveController::solve_3x3_gauss_jordan(const double M[3][3],
                                              const double y[3],
                                              double out[3]) {
  constexpr double kEps = 1e-9;

  // Augmented matrix [M | y]
  double Aaug[3][4] = {
      {M[0][0], M[0][1], M[0][2], y[0]},
      {M[1][0], M[1][1], M[1][2], y[1]},
      {M[2][0], M[2][1], M[2][2], y[2]},
  };

  for (int col = 0; col < 3; ++col) {
    int pivot = col;
    double best = std::abs(Aaug[col][col]);
    for (int r = col + 1; r < 3; ++r) {
      const double v = std::abs(Aaug[r][col]);
      if (v > best) {
        best = v;
        pivot = r;
      }
    }

    if (best < kEps) {
      return false;
    }

    if (pivot != col) {
      for (int c = col; c < 4; ++c) {
        std::swap(Aaug[col][c], Aaug[pivot][c]);
      }
    }

    const double inv_p = 1.0 / Aaug[col][col];
    for (int c = col; c < 4; ++c) {
      Aaug[col][c] *= inv_p;
    }

    for (int r = 0; r < 3; ++r) {
      if (r == col)
        continue;
      const double f = Aaug[r][col];
      if (std::abs(f) < kEps)
        continue;
      for (int c = col; c < 4; ++c) {
        Aaug[r][c] -= f * Aaug[col][c];
      }
    }
  }

  out[0] = Aaug[0][3];
  out[1] = Aaug[1][3];
  out[2] = Aaug[2][3];
  return true;
}

void SwerveController::update_odometry_and_publish_(const rclcpp::Time &time) {
  if (time < next_publish_timestamp_) {
    return;
  }

  next_publish_timestamp_ = time + publish_period_;
  std::array<double, kNumModules> vix{};
  std::array<double, kNumModules> viy{};

  if (axles_.size() != kNumModules) {
    RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "SwerveController: Expected %zu modules for odometry (got %zu)",
        kNumModules, axles_.size());
    return;
  }
  if (!read_axle_vectors(vix, viy)) {
    return;
  }
  // Solve Mx = y  ==>  x = M^-1 y
  double M[3][3] = {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}};
  double y[3] = {0.0, 0.0, 0.0};

  auto accumulate_row = [&](double a0, double a1, double a2, double b) {
    // M += a^T a
    M[0][0] += a0 * a0;
    M[0][1] += a0 * a1;
    M[0][2] += a0 * a2;
    M[1][0] += a1 * a0;
    M[1][1] += a1 * a1;
    M[1][2] += a1 * a2;
    M[2][0] += a2 * a0;
    M[2][1] += a2 * a1;
    M[2][2] += a2 * a2;
    // y += a^T b
    y[0] += a0 * b;
    y[1] += a1 * b;
    y[2] += a2 * b;
  };

  for (size_t i = 0; i < kNumModules; ++i) {
    accumulate_row(1.0, 0.0, -py_[i], vix[i]);
    accumulate_row(0.0, 1.0, +px_[i], viy[i]);
  }

  double out[3];
  if (!solve_3x3_gauss_jordan(M, y, out)) {
    return;
  }

  const auto &vx_meas = out[0];
  const auto &vy_meas = out[1];
  const auto &wz_meas = out[2];

  if (!realtime_odometry_publisher_ ||
      !realtime_odometry_publisher_->trylock()) {
    return;
  }

  auto &msg = realtime_odometry_publisher_->msg_;
  msg.header.stamp = time;
  msg.header.frame_id = odom_frame_id_;
  msg.child_frame_id = base_frame_id_;

  msg.twist.twist.linear.x = vx_meas;
  msg.twist.twist.linear.y = vy_meas;
  msg.twist.twist.linear.z = 0.0;
  msg.twist.twist.angular.x = 0.0;
  msg.twist.twist.angular.y = 0.0;
  msg.twist.twist.angular.z = wz_meas;

  std::fill(msg.twist.covariance.begin(), msg.twist.covariance.end(), 0.0);

  build_covariance_matrix(msg.twist.covariance, covariance_);

  realtime_odometry_publisher_->unlockAndPublish();
}

void SwerveController::on_message(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  geometry_msgs::msg::TwistStamped stamped;
  stamped.header.stamp = get_node()->now();
  stamped.header.frame_id = "base_link";
  stamped.twist = *msg;
  received_velocity_msg_.set(stamped);
}

void SwerveController::on_stamped_message(
    const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
  received_velocity_msg_.set(*msg);
}

} // namespace swerve_controller

PLUGINLIB_EXPORT_CLASS(swerve_controller::SwerveController,
                       controller_interface::ControllerInterface);