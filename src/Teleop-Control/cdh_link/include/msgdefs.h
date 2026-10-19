#ifndef MSGDEFS_H
#define MSGDEFS_H

#include <stdint.h>

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

enum NavStatus : int8_t { NO_FIX = -1, FIX = 0 };

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

#define APID_DRIVESTATUSMSG 1
#define APID_NAVINFOMSG 2
#define APID_SYSINFOMSG 3

#endif // MSGDEFS_H
