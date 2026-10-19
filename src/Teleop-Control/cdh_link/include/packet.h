#ifndef PACKET_H
#define PACKET_H

#include <cstring>
#include <stdint.h>

enum PacketType { TELEMETRY = 0, COMMAND = 1 };

class SpacePacket {
public:
  static void encode(uint8_t *buf, void *payload, uint16_t len, PacketType type,
                     uint16_t apid, uint16_t seq) {
    buf[0] = (type << 4) | ((apid & 0x7ff) >> 8);
    buf[1] = apid & 0xff;
    buf[2] = (0b11 << 6) | ((seq & 0x3fff) >> 8);
    buf[3] = seq & 0xff;
    buf[4] = (len & 0xff00) >> 8;
    buf[5] = len & 0xff;
    memcpy(buf + 6, payload, len);
  }

  static uint16_t getApid(uint8_t *buf) {
    return ((buf[0] & 0xf) << 8) | buf[1];
  }

  template <typename T>
  static int decode(uint8_t *buf, T *payload, PacketType *type, uint16_t *apid,
                    uint16_t *seq) {
    *type = static_cast<PacketType>(buf[0] & 0x1000);
    *apid = ((buf[0] & 0xf) << 8) | buf[1];
    *seq = ((buf[2] & 0x3f) << 8) | buf[3];
    uint16_t len = (buf[4] << 8) | buf[5];
    if (len == sizeof(T)) {
      memcpy(payload, buf + 6, len);
      return 0;
    } else {
      return -1;
    }
  }
};

#endif // PACKET_H
