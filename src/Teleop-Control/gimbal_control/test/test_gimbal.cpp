#include "gimbal_control/gimbal_controller.hpp"
#include "gimbal_control/maestro_system.hpp"
#include "pluginlib/class_loader.hpp"
#include <gtest/gtest.h>
#include <limits>
#include <poll.h>
#include <pty.h>
#include <unistd.h>

TEST(Plugins, Load) {
  pluginlib::ClassLoader<controller_interface::ControllerInterface> controllers(
      "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_NE(controllers.createSharedInstance("gimbal_control/GimbalController"),
            nullptr);
  pluginlib::ClassLoader<hardware_interface::SystemInterface> hardware(
      "hardware_interface", "hardware_interface::SystemInterface");
  EXPECT_NE(hardware.createSharedInstance("gimbal_control/MaestroSystem"),
            nullptr);
}

hardware_interface::HardwareInfo info(const std::string &port) {
  hardware_interface::HardwareInfo result;
  result.name = "TestMaestro";
  result.type = "system";
  result.hardware_parameters["serial_port"] = port;
  hardware_interface::ComponentInfo joint;
  joint.name = "pan";
  joint.type = "joint";
  hardware_interface::InterfaceInfo interface;
  interface.name = "position";
  interface.min = "-3.10";
  interface.max = "3.18";
  joint.command_interfaces.push_back(interface);
  joint.state_interfaces.push_back(interface);
  joint.parameters = {{"channel", "1"},    {"center", "3.10"},
                      {"range", "6.2832"}, {"min_us", "350"},
                      {"max_us", "2500"},  {"direction", "1"}};
  result.joints.push_back(joint);
  return result;
}

TEST(Maestro, RejectBadConfigurationAndMissingDevice) {
  gimbal_control::MaestroSystem system;
  auto config = info("/does/not/exist");
  config.joints[0].parameters["range"] = "nan";
  EXPECT_EQ(system.on_init(config), hardware_interface::CallbackReturn::ERROR);
  config = info("/does/not/exist");
  ASSERT_EQ(system.on_init(config),
            hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State()),
            hardware_interface::CallbackReturn::ERROR);
}

TEST(Maestro, SerialProtocolLimitsLifecycleAndDisconnect) {
  int master, slave;
  char path[256];
  ASSERT_EQ(openpty(&master, &slave, path, nullptr, nullptr), 0);
  gimbal_control::MaestroSystem system;
  ASSERT_EQ(system.on_init(info(path)),
            hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State()),
            hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
            hardware_interface::CallbackReturn::SUCCESS);
  auto expect_packet = [&](int target) {
    unsigned char packet[6]{};
    pollfd fd{master, POLLIN, 0};
    ASSERT_EQ(poll(&fd, 1, 200), 1);
    ASSERT_EQ(::read(master, packet, sizeof(packet)), 6);
    EXPECT_EQ(packet[0], 0xAA);
    EXPECT_EQ(packet[1], 12);
    EXPECT_EQ(packet[2], 4);
    EXPECT_EQ(packet[3], 1);
    EXPECT_EQ(packet[4] + (packet[5] << 7), target);
  };
  expect_packet(std::lround(4 * (350 + 2150 * 3.10 / 6.2832)));
  const rclcpp::Time now(0);
  const auto period = rclcpp::Duration::from_seconds(0.01);
  commands[0].set_value(-3.10);
  ASSERT_EQ(system.write(now, period), hardware_interface::return_type::OK);
  expect_packet(1400);
  EXPECT_DOUBLE_EQ(states[0].get_value(), -3.10);
  commands[0].set_value(3.18);
  ASSERT_EQ(system.write(now, period), hardware_interface::return_type::OK);
  expect_packet(std::lround(4 * (350 + 2150 * 6.28 / 6.2832)));
  for (double invalid :
       {3.19, -3.11, std::numeric_limits<double>::quiet_NaN()}) {
    commands[0].set_value(invalid);
    EXPECT_EQ(system.write(now, period),
              hardware_interface::return_type::ERROR);
    EXPECT_DOUBLE_EQ(states[0].get_value(), 3.18);
  }
  system.on_deactivate(rclcpp_lifecycle::State());
  commands[0].set_value(0.0);
  EXPECT_EQ(system.write(now, period), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(states[0].get_value(), 3.18);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State()),
            hardware_interface::CallbackReturn::SUCCESS);
  expect_packet(std::lround(4 * (350 + 2150 * 6.28 / 6.2832)));
  ::close(master);
  ::close(slave);
  commands[0].set_value(0.0);
  EXPECT_EQ(system.write(now, period), hardware_interface::return_type::ERROR);
  EXPECT_DOUBLE_EQ(states[0].get_value(), 3.18);
  system.on_cleanup(rclcpp_lifecycle::State());
}

class ControllerTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok())
      rclcpp::init(0, nullptr);
    controller = std::make_unique<gimbal_control::GimbalController>();
#ifdef GIMBAL_INIT_WITH_DESCRIPTION
    // Iron and newer require the robot description and manager update rate.
    ASSERT_EQ(controller->init("test_gimbal", "", 100, "",
                               controller->define_custom_node_options()),
              controller_interface::return_type::OK);
#else
    ASSERT_EQ(controller->init("test_gimbal"),
              controller_interface::return_type::OK);
#endif
  }
  std::unique_ptr<gimbal_control::GimbalController> controller;
};

TEST_F(ControllerTest, RejectInvalidParameters) {
  controller->get_node()->set_parameter(rclcpp::Parameter(
      "joints", std::vector<std::string>{"duplicate", "duplicate", "mast"}));
  EXPECT_EQ(controller->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
  controller->get_node()->set_parameter(rclcpp::Parameter(
      "joints", std::vector<std::string>{"pan", "tilt", "mast"}));
  controller->get_node()->set_parameter(
      rclcpp::Parameter("command_timeout", 0.0));
  EXPECT_EQ(controller->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
  controller->get_node()->set_parameter(
      rclcpp::Parameter("command_timeout", 0.5));
  controller->get_node()->set_parameter(
      rclcpp::Parameter("lower_limits", std::vector<double>{1.0, -1.0, -1.0}));
  EXPECT_EQ(controller->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
}

TEST_F(ControllerTest, ReorderedInterfacesHoldAndRejectInvalidState) {
  ASSERT_EQ(controller->on_configure(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
  const std::array<std::string, 3> names{
      "mast_camera_joint", "gimbal_pan_joint", "gimbal_tilt_joint"};
  std::array<double, 3> states{0.3, 0.1, -0.2}, commands{};
  std::vector<hardware_interface::CommandInterface> raw_commands;
  std::vector<hardware_interface::StateInterface> raw_states;
  for (size_t i = 0; i < 3; ++i) {
    raw_commands.emplace_back(names[i], "position", &commands[i]);
    raw_states.emplace_back(names[i], "position", &states[i]);
  }
  std::vector<hardware_interface::LoanedCommandInterface> loaned_commands;
  std::vector<hardware_interface::LoanedStateInterface> loaned_states;
  for (size_t i = 0; i < 3; ++i) {
    loaned_commands.emplace_back(raw_commands[i]);
    loaned_states.emplace_back(raw_states[2 - i]);
  }
  controller->assign_interfaces(std::move(loaned_commands),
                                std::move(loaned_states));
  ASSERT_EQ(controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(commands, states);
  for (double dt : {-1.0, 0.0, 0.01, 2.0}) {
    EXPECT_EQ(
        controller->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(dt)),
        controller_interface::return_type::OK);
    EXPECT_EQ(commands, states);
  }
  controller->on_deactivate(rclcpp_lifecycle::State());
  states[0] = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(controller->on_activate(rclcpp_lifecycle::State()),
            controller_interface::CallbackReturn::ERROR);
  controller->release_interfaces();
}
