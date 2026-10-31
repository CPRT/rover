#ifndef CDH_LINK_HPP_
#define CDH_LINK_HPP_

#include "rclcpp/rclcpp.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>

#include "packet.h"

namespace cdh_link {

template <typename T> struct TlmMsg {
  T msg{};
  uint16_t seq;
};

typedef std::function<void(uint8_t *payload, PacketType type, uint16_t apid,
                           uint16_t seq)>
    RecvCb;

class CdhLink {
public:
  CdhLink(rclcpp::Logger logger, char *prefix, RecvCb recv_cb)
      : logger(logger), prefix(prefix), recv_cb_(recv_cb){};
  ~CdhLink() {
    {
      // std::lock_guard<std::mutex> lock(mtx_);
      running_ = false;
    }
    // cv_.notify_all();
    if (recv_thread_ && recv_thread_->joinable()) {
      recv_thread_->join();
    }
    close(tm_fd_);
    close(tc_fd_);
  };

  void open(std::string tm_ip, uint16_t tm_port, uint16_t tc_port) {
    this->tm_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->tm_fd_ < 0) {
      RCLCPP_FATAL(this->logger, "[%s] TM Socket error %u", prefix, errno);
    }

    tm_addr_.sin_family = AF_INET;
    tm_addr_.sin_port = htons(tm_port);
    if (inet_aton(tm_ip.c_str(), &tm_addr_.sin_addr) == 0) {
      RCLCPP_FATAL(this->logger, "[%s] Error converting address %s", prefix,
                   tm_ip.c_str());
    }

    this->tc_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->tc_fd_ < 0) {
      RCLCPP_FATAL(this->logger, "[%s] TC Socket error %s", prefix,
                   strerror(errno));
    }

    tc_addr_.sin_family = AF_INET;
    tc_addr_.sin_addr.s_addr = INADDR_ANY;
    tc_addr_.sin_port = htons(tc_port);

    if (bind(this->tc_fd_, (struct sockaddr *)&this->tc_addr_,
             sizeof(this->tc_addr_)) != 0) {
      RCLCPP_FATAL(this->logger, "[%s] TM Socket error %s", prefix,
                   strerror(errno));
    }

    running_ = true;
    recv_thread_ = std::make_shared<std::thread>([this]() {
      while (true) {
        if (!this->running_) {
          break;
        }
        uint8_t buf[1024];
        int recvd = recvfrom(this->tc_fd_, buf, sizeof(buf), 0, NULL, NULL);
        uint8_t payload[1024];
        PacketType type;
        uint16_t apid, seq;
        if (SpacePacket::decode(payload, buf, recvd, &type, &apid, &seq) == 0) {
          this->recv_cb_(payload, type, apid, seq);
        } else {
          RCLCPP_WARN(this->logger,
                      "[%s] Packet length error: expected %u, received %u",
                      prefix, (buf[4] << 8) | buf[5], recvd - 6);
        }
      }
    });

    RCLCPP_INFO(this->logger,
                "[%s] Link started, TM out: %s:%u, listening on :%u", prefix,
                tm_ip.c_str(), tm_port, tc_port);
  }

  template <typename T> void sendTlm(TlmMsg<T> msg, uint16_t apid) {
    uint8_t buf[sizeof(T) + 6];
    SpacePacket::encode(buf, &msg.msg, sizeof(msg.msg), PacketType::TELEMETRY,
                        apid, msg.seq);
    sendto(this->tm_fd_, buf, sizeof(buf), 0,
           (struct sockaddr *)&this->tm_addr_, sizeof(this->tm_addr_));
  }

private:
  int tm_fd_;
  int tc_fd_;
  struct sockaddr_in tm_addr_ {};
  struct sockaddr_in tc_addr_ {};

  std::shared_ptr<std::thread> recv_thread_;
  std::atomic_bool running_;
  RecvCb recv_cb_;

  rclcpp::Logger logger;
  char *prefix;
};

} // namespace cdh_link

#endif // CDH_LINK_HPP_