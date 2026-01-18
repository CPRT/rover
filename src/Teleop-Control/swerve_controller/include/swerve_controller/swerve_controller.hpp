#pragma once

#include <array>
#include <cmath>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_box.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "swerve_controller/axle.hpp"

namespace swerve_controller {

constexpr static size_t kNumModules = 4;

/**
 * @brief ROS 2 controller for a 4-module swerve drive.
 *
 * Responsibilities:
 *  - Accept commanded body twist (vx, vy, wz)
 *  - Command individual swerve modules
 *  - Compute wheel-based odometry twist using least-squares kinematics
 *  - Publish nav_msgs/Odometry with computed twist
 */
class SwerveController : public controller_interface::ControllerInterface {
public:
  SwerveController() = default;

  /**
   * @brief Declare command interfaces required by this controller.
   */
  controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;

  /**
   * @brief Declare state interfaces required by this controller.
   */
  controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  /**
   * @brief Controller initialization (parameter declaration).
   */
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Configure controller (read parameters, setup
   * publishers/subscribers).
   */
  controller_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Activate controller (start realtime publishers).
   */
  controller_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Deactivate controller.
   */
  controller_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Cleanup resources.
   */
  controller_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Error handling hook.
   */
  controller_interface::CallbackReturn
  on_error(const rclcpp_lifecycle::State &previous_state) override;

  /**
   * @brief Main control update loop.
   *
   * Called at the controller update rate. Handles:
   *  - Reading latest command
   *  - Computing swerve kinematics
   *  - Publishing wheel-based odometry
   */
  controller_interface::return_type
  update(const rclcpp::Time &time, const rclcpp::Duration &period) override;

private:
  // ---------------------------------------------------------------------------
  // Command callbacks
  // ---------------------------------------------------------------------------

  /**
   * @brief Handle unstamped velocity commands.
   */
  void on_message(const geometry_msgs::msg::Twist::SharedPtr msg);

  /**
   * @brief Handle stamped velocity commands.
   */
  void
  on_stamped_message(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

  // ---------------------------------------------------------------------------
  // Odometry helpers
  // ---------------------------------------------------------------------------

  /**
   * @brief Read wheel velocity vectors in the base frame.
   *
   * For each swerve module, converts steering angle + wheel speed
   * into an (x,y) velocity vector expressed in the robot base frame.
   *
   * @param[out] x_components Module x-velocity components [m/s]
   * @param[out] y_components Module y-velocity components [m/s]
   * @return true if all modules were read successfully
   */
  bool read_axle_vectors(std::array<double, kNumModules> &x_components,
                         std::array<double, kNumModules> &y_components);

  /**
   * @brief Build a 6-DOF diagonal covariance matrix from a vector.
   *
   * Maps a 6-element covariance vector onto the diagonal entries
   * of a 6x6 covariance matrix (stored as a 36-element array).
   *
   * Ordering follows nav_msgs/Odometry conventions.
   */
  void build_covariance_matrix(std::array<double, 36> &cov_matrix,
                               const std::vector<double> &cov_vector);

  /**
   * @brief Compute and publish wheel-based odometry.
   *
   * Uses a least-squares rigid-body solve to estimate body twist:
   *   [vx, vy, wz]
   *
   * Pose is NOT integrated here; only twist is published.
   */
  void update_odometry_and_publish_(const rclcpp::Time &time);

  /**
   * @brief Solve a 3x3 linear system using Gauss-Jordan elimination.
   *
   * Solves:
   *   M * x = y
   *
   * Used internally for least-squares odometry.
   *
   * @param[in]  M   3x3 matrix
   * @param[in]  y   RHS vector
   * @param[out] out Solution vector
   * @return true if the matrix was non-singular
   */
  bool solve_3x3_gauss_jordan(const double M[3][3], const double y[3],
                              double out[3]);

  /**
   * @brief Check if a steering angle is within valid limits.
   * @param angle Steering angle [rad]
   * @return true if angle is valid
   */
  double isValidAngle(double angle);

  /**
   * @brief Optimize steering angle and wheel speed to minimize rotation.
   *
   * If the required steering angle change is greater than 90 degrees,
   * this function adjusts the target angle by adding/subtracting 180 degrees
   * and inverts the wheel speed to achieve the same net motion with less
   * rotation.
   *
   * @param current_angle Current steering angle [rad]
   * @param target_angle Target steering angle [rad] (modified in-place)
   * @param target_wheel_rad_s Target wheel speed [rad/s] (modified in-place)
   */
  void optimize_steering(double current_angle, double &target_angle,
                         double &target_wheel_rad_s);

  // ---------------------------------------------------------------------------
  // ROS interfaces
  // ---------------------------------------------------------------------------

  /// Subscription for unstamped velocity commands
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

  /// Subscription for stamped velocity commands
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr
      stamped_subscription_;

  /// Standard odometry publisher
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Odometry>>
      odometry_publisher_ = nullptr;

  /// Realtime odometry publisher
  std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::msg::Odometry>>
      realtime_odometry_publisher_ = nullptr;

  /// Pre-allocated odometry message for realtime publishing
  nav_msgs::msg::Odometry odometry_message_;

  /// Thread-safe storage for latest received command
  realtime_tools::RealtimeBox<geometry_msgs::msg::TwistStamped>
      received_velocity_msg_;

  // ---------------------------------------------------------------------------
  // Swerve hardware
  // ---------------------------------------------------------------------------

  /**
   * @brief Swerve module list in fixed order:
   *  [0] Front-Left
   *  [1] Front-Right
   *  [2] Rear-Left
   *  [3] Rear-Right
   */
  std::vector<Axle> axles_;

  inline Axle &front_left_axle_() { return axles_[0]; }
  inline Axle &front_right_axle_() { return axles_[1]; }
  inline Axle &rear_left_axle_() { return axles_[2]; }
  inline Axle &rear_right_axle_() { return axles_[3]; }

  // ---------------------------------------------------------------------------
  // Parameters
  // ---------------------------------------------------------------------------

  double wheel_base_;   ///< Distance between front and rear axles [m]
  double track_width_;  ///< Distance between left and right wheels [m]
  double wheel_radius_; ///< Wheel radius [m]
  double min_angle_;    ///< Minimum steering angle [rad]
  double max_angle_;    ///< Maximum steering angle [rad]
  double cmd_timeout_;  ///< Command timeout [s]

  std::string odom_frame_id_; ///< Odometry frame
  std::string base_frame_id_; ///< Robot base frame

  double max_linear_velocity_;  ///< Velocity command limit [m/s]
  double max_angular_velocity_; ///< Angular velocity limit [rad/s]

  /// Diagonal covariance values for odometry (size must be 6)
  std::vector<double> covariance_;

  /// Module x-positions relative to base_link
  std::array<double, kNumModules> px_;

  /// Module y-positions relative to base_link
  std::array<double, kNumModules> py_;

  // ---------------------------------------------------------------------------
  // Publishing timing
  // ---------------------------------------------------------------------------

  double odom_publish_rate_{50.0}; ///< Odometry publish rate [Hz]
  rclcpp::Time next_publish_timestamp_{0, 0, RCL_CLOCK_UNINITIALIZED};
  rclcpp::Duration publish_period_{0, 0};
};

} // namespace swerve_controller
