#ifndef ARM_HPP
#define ARM_HPP

#include "arm_control/MoveGroupClient.hpp"
#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "interfaces/srv/get_poses.hpp"
#include "interfaces/srv/go_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_control.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/trigger.hpp"

class arm : public rclcpp::Node {
public:
  arm();

private:
  void arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void manual_arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void ik_arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void ik_pose_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void endeffector_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void clipboards_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  bool moveit_servo_state(bool enable);
  void clear_dot();
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ik_pub_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr eef_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr dot_pub_;
  rclcpp::Service<interfaces::srv::GetPoses>::SharedPtr name_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<interfaces::srv::GoToPose>::SharedPtr go_to_pose_service_;
  std::shared_ptr<arm_control::MoveGroupClient> moveit_client_;
  enum ArmState { NONE = 0, MANUAL, IK, POS };

  ArmState current_state_;

  void declareParameters();
  void loadParameters();
  geometry_msgs::msg::PoseStamped getPoseOfPtr();
  bool initialized_ = false;

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
  int clipboard1_pose_index_ = -1;
  int clipboard2_pose_index_ = -1;
  std::shared_ptr<sensor_msgs::msg::Joy> last_msg_;
};

#endif