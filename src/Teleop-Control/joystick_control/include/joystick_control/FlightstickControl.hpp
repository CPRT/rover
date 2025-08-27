#ifndef FLIGHTSTICK_CONTROL_HPP
#define FLIGHTSTICK_CONTROL_HPP

#include "ArmDummyMode.hpp"
#include "ArmIKMode.hpp"
#include "ArmManualMode.hpp"
#include "DriveMode.hpp"
#include "ScienceMode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int8.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @class FlightstickControl
 * @brief A class to handle joystick inputs and control modes for the rover.
 *
 * The FlightstickControl class subscribes to joystick messages and processes
 * them to control different modes of the rover, such as driving, arm control,
 * navigation, and science operations.
 */
class FlightstickControl : public rclcpp::Node {
 public:
  /**
   * @brief Constructor for FlightstickControl.
   *
   * Initializes the node and sets up the joystick subscription.
   */
  FlightstickControl();

  /**
   * @brief Processes joystick messages.
   *
   * This function is called whenever a new joystick message is received. It
   * processes the joystick inputs and performs actions based on the current
   * mode.
   *
   * @param joystickMsg A shared pointer to the received joystick message.
   */
  void processJoystick(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

 private:
  /*
   * @brief Declares parameters for the node and modes.
   */
  void declareParameters();
  /**
   * @brief Loads parameters from the parameter server.
   *
   * This function loads configuration parameters such as button mappings from
   * the parameter server.
   */
  void loadParameters();

  /**
   * @enum ModeType
   * @brief Enumeration of the different control modes.
   *
   * This enumeration defines the different control modes that the rover can be
   * in.
   */
  enum class ModeType {
    NONE = 0,
    DRIVE,
    ARM_IK,
    ARM_MANUAL,
    NAV,
    SCIENCE,
    ARM_DUMMY
  };

  uint8_t kDriveModeButton;      ///< Button index for drive mode.
  uint8_t kArmIKModeButton;      ///< Button index for inverse kinematics arm
                                 ///< control mode.
  uint8_t kArmManualModeButton;  ///< Button index for manual arm control mode.
  uint8_t kNavModeButton;        ///< Button index for navigation mode.
  uint8_t kScienceModeButton;    ///< Button index for science mode.
  uint8_t kArmDummyButton;

  uint8_t kTeleopLightMode;  ///< Button index for teleoperation light mode.

  ModeType currentMode_;  ///< The current control mode of the rover.

  /**
   * @brief Checks for mode change based on joystick input.
   *
   * This function checks if a mode change is requested based on the joystick
   * input.
   *
   * @param joystickMsg A shared pointer to the received joystick message.
   * @return True if a mode change is requested, false otherwise.
   */
  bool checkForModeChange(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg);

  /**
   * @brief Checks all axis and button values
   *
   * This function checks if all relevant axis and button values are 0
   *
   * @param joystickMsg A shared pointer to the received joystick message.
   * @param nextMode The mode to check against.
   * @return True if all axes (except slider) are 0, false otherwise
   */
  bool checkAxes(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg,
                 ModeType nextMode);

  /**
   * @brief Changes the control mode.
   *
   * This function changes the control mode of the rover to the specified mode.
   *
   * @param mode The new control mode to switch to.
   * @return True if the mode change was successful, false otherwise.
   */
  bool changeMode(ModeType mode);

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr
      joy_sub_;  ///< Subscription to joystick messages.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
      status_pub_;  ///< Publisher for status messages.
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr
      light_pub_;  ///< Publisher for light control messages.

  std::unique_ptr<Mode> mode_;  ///< Pointer to the current mode object.
};

#endif  // FLIGHTSTICK_CONTROL_HPP
