#ifndef ARM_HPP
#define ARM_HPP

#include "control_msgs/msg/joint_jog.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "interfaces/action/move_to_pose.hpp"
#include "interfaces/srv/move_servo.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

using MoveToPose = interfaces::action::MoveToPose;
using GoalHandleMoveToPose = rclcpp_action::ClientGoalHandle<MoveToPose>;
class arm : public rclcpp::Node {
public:
  arm();
  void arm_control(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

  void send_goal_pose(geometry_msgs::msg::Pose pose);

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr ik_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  control_msgs::msg::JointJog joint_msg_;
  sensor_msgs::msg::Joy::SharedPtr last_joy_msg_;

  void arm_manual(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void arm_ik(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void check_preset(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void mode_switch(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

  rclcpp_action::Client<MoveToPose>::SharedPtr move_group_action_client_;

  void
  movegroup_goal_response_callback(GoalHandleMoveToPose::SharedPtr goal_handle);
  void movegroup_feedback_callback(
      GoalHandleMoveToPose::SharedPtr,
      const std::shared_ptr<const MoveToPose::Feedback> feedback);
  void
  movegroup_result_callback(const GoalHandleMoveToPose::WrappedResult &result);
  void request_position(std::string pose_id);

  bool button_pressed(int index,
                      const sensor_msgs::msg::Joy::SharedPtr &current);

  std::map<std::string, std::vector<double>> positions;

  bool is_moving_ = false;

  int mode_button_value = 0;
  int mode_button_value_prev = 0;
  int modes = 3;
  int mode = 1;

  int kHomeButton;
  int kStowButton;

  void declareParameters();
  void loadParameters();

  int mode_switch_button;

  double kMaxBaseSpeed;
  double kMaxWristRollSpeed;
  double kMaxWristSpeed;
  double kMaxAct1Speed;
  double kMaxAct2Speed;
  double kMaxElbowYawSpeed;
  int kThrottleAxis;
  int kBaseAxis;
  int kWristRoll;
  int kWristYaw_positive;
  int kWristYaw_negative;
  int kAct1Axis;
  int kAct2Axis;
  int kElbowYaw;

  int kForwardAxis;
  int kLateralAxis;
  int kVerticalAxis;
  int kRollAxis;
  int kPitchAxis;
  int kYawAxis;

  double kMaxForwardSpeed;
  double kMaxLateralSpeed;
  double kMaxVerticalSpeed;
  double kMaxRollSpeed;
  double kMaxPitchSpeed;
  double kMaxYawSpeed;

  int kGripperButton;
};

#endif