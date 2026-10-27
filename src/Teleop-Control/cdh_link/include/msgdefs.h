#ifndef MSGDEFS_H
#define MSGDEFS_H

#include <stdint.h>

enum NavStatus : int8_t { NO_FIX = -1, FIX = 0 };
enum ArmMode : uint8_t { OFF, IDLE, IK, JOG };

struct MotorStatus {
  float bus_voltage;
  float current;
  double position;
  double velocity;
  uint32_t errors;
};

struct DriveStatusMsg {
  MotorStatus motors[8];
};

struct NavInfoMsg {
  double latitude;
  double longitude;
  double altitude;
  double heading;
  double variances[3];
  NavStatus status;
};

struct SysInfoMsg {
  uint8_t cpu_usage;
  uint8_t mem_usage;
  uint8_t gpu_usage;
  uint8_t core_temp;
};

struct ArmStatusMsg {
  float x;
  float y;
  float z;
  float roll;
  float pitch;
  float yaw;
  float eef;
  MotorStatus joints[7];
  ArmMode mode;
};

#define APID_DRIVESTATUSMSG 1
#define APID_NAVINFOMSG 2
#define APID_SYSINFOMSG 3

struct DriveCtrlCmd {
  int16_t x;
  int16_t y;
  int16_t yaw;
  int16_t throttle;
};

struct ArmJogCmd {
  int16_t joint1;
  int16_t joint2;
  int16_t joint3;
  int16_t joint4;
  int16_t joint5;
  int16_t joint6;
  int16_t eef;
  int16_t throttle;
};

struct ArmSetModeCmd {
  ArmMode mode;
};

#define APID_DRIVECTRLCMD 1
#define APID_ARMJOGCMD 2
#define APID_ARMSETMODECMD 3

std::ostream &operator<<(std::ostream &os, const SysInfoMsg &msg) {
  os << "CPU: " << (int)msg.cpu_usage << "% GPU: " << (int)msg.gpu_usage
     << "% Mem: " << (int)msg.mem_usage << "% Core: " << (int)msg.core_temp
     << " deg C";
  return os;
}

std::ostream &operator<<(std::ostream &os, const NavInfoMsg &msg) {
  os << "Lat: " << msg.latitude << " Long: " << msg.longitude
     << " Alt: " << msg.altitude << " Head: " << msg.heading
     << " Var: " << msg.variances[0] << ", " << msg.variances[1] << ", "
     << msg.variances[2] << " Status: " << static_cast<int>(msg.status);
  return os;
}

std::ostream &operator<<(std::ostream &os, const MotorStatus &msg) {
  os << "Bus Voltage: " << msg.bus_voltage << " Current: " << msg.current
     << " Position: " << msg.position << " Velocity: " << msg.velocity
     << " Errors: " << msg.errors << std::endl;
  return os;
}

std::ostream &operator<<(std::ostream &os, const DriveStatusMsg &msg) {
  for (int i = 0; i < 8; i++) {
    os << msg.motors[i];
  }
  return os;
}

#endif // MSGDEFS_H
