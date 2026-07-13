#ifndef ARM_HPP
#define ARM_HPP

#include "control_msgs/msg/joint_jog.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "interfaces/srv/go_to_cam_coord.hpp"
#include "interfaces/srv/go_to_named_pose.hpp"
#include "interfaces/srv/save_current_pose.hpp"
#include "moveit_msgs/srv/servo_command_type.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_control.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace joystick_control {

class ArmTeleop : public rclcpp::Node {
public:
  explicit ArmTeleop(const rclcpp::NodeOptions &options);
  ~ArmTeleop();

private:
  enum ArmState {
    NO_MESSAGE = 0,
    UNPLUG_ERROR,
    WIGGLE_WARNING,
    IDLE,
    MANUAL,
    IK,
    POS
  };

  void manual_arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void ik_arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void ik_pose_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void endeffector_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void clipboards_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void clear_dot();
  ArmState
  check_initialized(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  bool moveit_servo_configure(const ArmState requested_state);
  ArmState requested_state(const std::vector<int32_t> &buttons) const;
  bool switch_states(const ArmState new_state);
  bool stop_move_group_motion();
  bool go_to_named_pose(const std::string &name);
  bool save_current_pose(const std::string &name);
  bool go_to_cam_coord(double u, double v);
  void run();
  void declareParameters();
  void loadParameters();
  void publishState(const ArmState state);
  bool is_ready(const ArmState state);
  static std::string state_to_string(const ArmState state);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ik_pub_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr eef_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr dot_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;

  rclcpp::Client<interfaces::srv::GoToNamedPose>::SharedPtr
      go_to_named_pose_client_;
  rclcpp::Client<interfaces::srv::SaveCurrentPose>::SharedPtr
      save_current_pose_client_;
  rclcpp::Client<interfaces::srv::GoToCamCoord>::SharedPtr
      go_to_cam_coord_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_move_group_client_;
  rclcpp::Client<moveit_msgs::srv::ServoCommandType>::SharedPtr
      servo_input_client_;

  ArmState current_state_;

  int kThrottleAxis;
  int kJoint1Axis;
  int kJoint2Axis;
  int kJoint3Axis;
  int kJoint4Axis;
  int kJoint6Axis;
  int kWristYaw_positive;
  int kWristYaw_negative;
  int kDisableButton;
  int kIkButton;
  int kManualButton;
  int kPositionButton;
  int kClawOpen;
  int kClawClose;
  int kCamWidth;
  int kCamHeight;
  int kDotInc;
  int kClipboard1SaveButton;
  int kClipboard1ExecuteButton;
  int kClipboard2SaveButton;
  int kClipboard2ExecuteButton;

  double targetPositionX;
  double targetPositionY;

  std::shared_ptr<sensor_msgs::msg::Joy> curr_msg_;
  std::shared_ptr<sensor_msgs::msg::Joy> last_msg_;
  std::shared_ptr<std::thread> run_thread_;
  std::atomic_bool running_;
  std::mutex mtx_;
  std::condition_variable cv_;
};

} // namespace joystick_control

#endif