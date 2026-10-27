#include <iomanip>
#include <iostream>

#include "msgdefs.h"
#include "packet.h"

#include <arpa/inet.h>
#include <sys/socket.h>

#define PARSE_MSG(apidsel, msgtype)                                            \
  case apidsel: {                                                              \
    msgtype *msgtype##_msg = reinterpret_cast<msgtype *>(payload);             \
    std::cout << "Seq: " << seq << " " << *msgtype##_msg << std::endl;         \
  } break;

int main(int argc, char *argv[]) {
  int fd;

  struct sockaddr_in si, other{};
  si.sin_family = AF_INET;
  si.sin_port = htons(1234);
  si.sin_addr.s_addr = htonl(INADDR_ANY);

  fd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  bind(fd, (struct sockaddr *)&si, sizeof(si));

  while (true) {
    uint8_t buf[1024];
    unsigned int slen = sizeof(other);
    int len = recvfrom(fd, buf, 1024, 0, (struct sockaddr *)&other, &slen);

    PacketType type;
    uint16_t apid, seq;
    uint8_t payload[1024];
    SpacePacket::decode(payload, buf, len, &type, &apid, &seq);

    if (type != PacketType::TELEMETRY) {
      std::cout << "Not telemetry" << std::endl;
      continue;
    }

    switch (apid) {
      PARSE_MSG(APID_DRIVESTATUSMSG, DriveStatusMsg)
      PARSE_MSG(APID_NAVINFOMSG, NavInfoMsg)
      PARSE_MSG(APID_SYSINFOMSG, SysInfoMsg)

    default:
      break;
    }
  }

  return 0;
}
