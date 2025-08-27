#ifndef JOYSTICK_CONTROL__ARMIK_MODE_HPP_
#define JOYSTICK_CONTROL__ARMIK_MODE_HPP_

#include <geometry_msgs/msg/twist_stamped.hpp>

#include "Mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/srv/move_servo.hpp"
#include "rclcpp/rclcpp.hpp"

const std::string CAM_FRAME_ID = "Link_6";
const std::string BASE_FRAME_ID = "Link_2";

class ArmIKMode : public Mode {
 public:
  ArmIKMode(rclcpp::Node* node);

  void processJoystickInput(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;

  void handleTwist(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);
  void handleGripper(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

  static void declareParameters(rclcpp::Node* node);
  void loadParameters();

  /**
   * @brief Sends a request to move a servo
   * @param port The servo port number
   * @param pos The target position
   * @return The service response
   */
  interfaces::srv::MoveServo::Response sendRequest(int port, int pos) const;

  /**
   * @brief Wrapper for servo control
   * @param req_port The servo port number
   * @param req_pos The target position
   */
  void servoRequest(int req_port, int req_pos) const;

 private:
  // Servo Members
  rclcpp::Client<interfaces::srv::MoveServo>::SharedPtr servo_client_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;

  // Servo Constants
  int8_t kServoPort;
  int8_t kServoMin;
  int8_t kServoMax;
  int8_t kClawMax;
  int8_t kClawMin;
  int8_t kxAxis;
  int8_t kyAxis;
  int8_t kUpBut;
  int8_t kDownBut;
  int8_t kAroundX;
  int8_t kAroundY;
  int8_t kAroundZ;
  int8_t kBase;
  int8_t kEEF;
  int8_t kClawOpen;
  int8_t kClawClose;
  int8_t servoPos_;
  bool buttonPressed_;
  bool swapButton_;
  std::string frame_to_publish_;
};

#endif  // JOYSTICK_CONTROL__ARMIK_MODE_HPP_