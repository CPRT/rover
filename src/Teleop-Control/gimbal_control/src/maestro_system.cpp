#include "gimbal_control/maestro_system.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <set>
#include <stdexcept>
#include <sys/file.h>
#include <termios.h>
#include <unistd.h>

namespace gimbal_control {
using CallbackReturn = hardware_interface::CallbackReturn;
MaestroSystem::~MaestroSystem() { close_port(); }
void MaestroSystem::close_port() {
  active_ = false;
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}
CallbackReturn
MaestroSystem::on_init(const hardware_interface::HardwareInfo &info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;
  try {
    port_ = info_.hardware_parameters.at("serial_port");
    if (port_.empty())
      throw std::invalid_argument("empty serial port");
    if (info_.hardware_parameters.count("device_number"))
      device_ = std::stoi(info_.hardware_parameters.at("device_number"));
    if (device_ < 0 || device_ > 127 || info_.joints.empty())
      throw std::invalid_argument("invalid device or joints");
    std::set<int> channels;
    servos_.clear();
    for (const auto &joint : info_.joints) {
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name != "position" ||
          joint.state_interfaces.size() != 1 ||
          joint.state_interfaces[0].name != "position")
        throw std::invalid_argument(
            "expected one position command/state per joint");
      const auto &p = joint.parameters;
      Servo s{std::stoi(p.at("channel")),
              std::stod(joint.command_interfaces[0].min),
              std::stod(joint.command_interfaces[0].max),
              std::stod(p.at("center")),
              std::stod(p.at("range")),
              std::stod(p.at("min_us")),
              std::stod(p.at("max_us")),
              std::stod(p.at("direction"))};
      for (double v : {s.lower, s.upper, s.center, s.range, s.min_us, s.max_us,
                       s.direction})
        if (!std::isfinite(v))
          throw std::invalid_argument("nonfinite calibration");
      if (s.channel < 0 || s.channel > 23 ||
          !channels.insert(s.channel).second || s.lower >= s.upper ||
          s.lower > 0 || s.upper < 0 || s.range <= 0 || s.min_us <= 0 ||
          s.min_us >= s.max_us || s.max_us > 4095.75 ||
          std::abs(s.direction) != 1 ||
          std::min(s.center + s.direction * s.lower,
                   s.center + s.direction * s.upper) < 0 ||
          std::max(s.center + s.direction * s.lower,
                   s.center + s.direction * s.upper) > s.range)
        throw std::invalid_argument("invalid servo calibration or limits");
      servos_.push_back(s);
    }
    commands_.assign(servos_.size(), 0);
    positions_.assign(servos_.size(), 0);
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("MaestroSystem"), "Configuration: %s",
                 e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
CallbackReturn MaestroSystem::on_configure(const rclcpp_lifecycle::State &) {
  close_port();
  fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    RCLCPP_ERROR(rclcpp::get_logger("MaestroSystem"), "Cannot open %s: %s",
                 port_.c_str(), strerror(errno));
    return CallbackReturn::ERROR;
  }
  termios config{};
  if (flock(fd_, LOCK_EX | LOCK_NB) != 0 || tcgetattr(fd_, &config) != 0) {
    close_port();
    return CallbackReturn::ERROR;
  }
  cfmakeraw(&config);
  cfsetispeed(&config, B9600);
  cfsetospeed(&config, B9600);
  config.c_cflag |= CLOCAL | CREAD;
  config.c_cflag &= ~CRTSCTS;
  if (tcsetattr(fd_, TCSANOW, &config) != 0) {
    close_port();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}
bool MaestroSystem::send_target(size_t i, double position) {
  const auto &s = servos_[i];
  const double angle = s.center + s.direction * position;
  const int target = static_cast<int>(
      std::lround(4 * (s.min_us + (s.max_us - s.min_us) * angle / s.range)));
  const std::array<unsigned char, 6> packet{
      {0xAA, static_cast<unsigned char>(device_), 0x04,
       static_cast<unsigned char>(s.channel),
       static_cast<unsigned char>(target & 0x7F),
       static_cast<unsigned char>((target >> 7) & 0x7F)}};
  size_t offset = 0;
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms_);
  while (offset < packet.size()) {
    const auto remaining =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            deadline - std::chrono::steady_clock::now())
            .count();
    if (remaining <= 0)
      return false;
    pollfd pfd{fd_, POLLOUT, 0};
    int result = ::poll(&pfd, 1, static_cast<int>(remaining));
    if (result < 0 && errno == EINTR)
      continue;
    if (result <= 0 || (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)))
      return false;
    const auto count =
        ::write(fd_, packet.data() + offset, packet.size() - offset);
    if (count < 0 && (errno == EINTR || errno == EAGAIN))
      continue;
    if (count <= 0)
      return false;
    offset += static_cast<size_t>(count);
  }
  return true;
}
CallbackReturn MaestroSystem::on_activate(const rclcpp_lifecycle::State &) {
  if (fd_ < 0)
    return CallbackReturn::ERROR;
  // Open-loop servos have no encoder feedback. Retain last commanded target on
  // reactivation.
  std::copy(positions_.begin(), positions_.end(), commands_.begin());
  for (size_t i = 0; i < servos_.size(); ++i)
    if (!send_target(i, commands_[i])) {
      close_port();
      return CallbackReturn::ERROR;
    }
  active_ = true;
  return CallbackReturn::SUCCESS;
}
CallbackReturn MaestroSystem::on_deactivate(const rclcpp_lifecycle::State &) {
  active_ =
      false; // Hold current pulse widths; do not drop the camera under gravity.
  return CallbackReturn::SUCCESS;
}
CallbackReturn MaestroSystem::on_cleanup(const rclcpp_lifecycle::State &) {
  close_port();
  return CallbackReturn::SUCCESS;
}
CallbackReturn MaestroSystem::on_shutdown(const rclcpp_lifecycle::State &) {
  close_port();
  return CallbackReturn::SUCCESS;
}
CallbackReturn MaestroSystem::on_error(const rclcpp_lifecycle::State &) {
  close_port();
  return CallbackReturn::SUCCESS;
}
std::vector<hardware_interface::StateInterface>
MaestroSystem::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> result;
  for (size_t i = 0; i < servos_.size(); ++i)
    result.emplace_back(info_.joints[i].name, "position", &positions_[i]);
  return result;
}
std::vector<hardware_interface::CommandInterface>
MaestroSystem::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> result;
  for (size_t i = 0; i < servos_.size(); ++i)
    result.emplace_back(info_.joints[i].name, "position", &commands_[i]);
  return result;
}
hardware_interface::return_type MaestroSystem::read(const rclcpp::Time &,
                                                    const rclcpp::Duration &) {
  // State is the last successfully transmitted target, not a measured shaft
  // angle.
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type MaestroSystem::write(const rclcpp::Time &,
                                                     const rclcpp::Duration &) {
  if (!active_)
    return hardware_interface::return_type::OK;
  for (size_t i = 0; i < servos_.size(); ++i)
    if (!std::isfinite(commands_[i]) || commands_[i] < servos_[i].lower ||
        commands_[i] > servos_[i].upper)
      return hardware_interface::return_type::ERROR;
  for (size_t i = 0; i < servos_.size(); ++i) {
    if (commands_[i] == positions_[i])
      continue;
    if (!send_target(i, commands_[i])) {
      RCLCPP_ERROR(rclcpp::get_logger("MaestroSystem"),
                   "Servo serial write failed");
      close_port();
      return hardware_interface::return_type::ERROR;
    }
    positions_[i] = commands_[i];
  }
  return hardware_interface::return_type::OK;
}
} // namespace gimbal_control
PLUGINLIB_EXPORT_CLASS(gimbal_control::MaestroSystem,
                       hardware_interface::SystemInterface)
