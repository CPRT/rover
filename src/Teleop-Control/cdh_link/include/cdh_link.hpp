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

enum LinkId {
  L900,
  L2400,
};

struct Link {
  LinkId id;
  std::string ground_ip;
  uint16_t hb_count_;
  hb_time_;
  bool alive;
  uint32_t bw_down;
  uint32_t bw_up;
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
    close(tx_fd_);
    close(rx_fd_);
  };

  void open(std::string tx_ip, uint16_t tx_port, uint16_t rx_port) {
    this->tx_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->tx_fd_ < 0) {
      RCLCPP_FATAL(this->logger, "[%s] TX Socket error %u", prefix, errno);
    }

    tx_addr_.sin_family = AF_INET;
    tx_addr_.sin_port = htons(tx_port);
    if (inet_aton(tx_ip.c_str(), &tx_addr_.sin_addr) == 0) {
      RCLCPP_FATAL(this->logger, "[%s] Error converting address %s", prefix,
                   tx_ip.c_str());
    }

    this->rx_fd_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->rx_fd_ < 0) {
      RCLCPP_FATAL(this->logger, "[%s] RX Socket error %s", prefix,
                   strerror(errno));
    }

    rx_addr_.sin_family = AF_INET;
    rx_addr_.sin_addr.s_addr = INADDR_ANY;
    rx_addr_.sin_port = htons(rx_port);

    if (bind(this->rx_fd_, (struct sockaddr *)&this->rx_addr_,
             sizeof(this->rx_addr_)) != 0) {
      RCLCPP_FATAL(this->logger, "[%s] tx Socket error %s", prefix,
                   strerror(errno));
    }

    running_ = true;
    recv_thread_ = std::make_shared<std::thread>([this]() {
      while (true) {
        if (!this->running_) {
          break;
        }
        uint8_t buf[1024];
        int recvd = recvfrom(this->rx_fd_, buf, sizeof(buf), 0, NULL, NULL);
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
                "[%s] Link started, tx out: %s:%u, listening on :%u", prefix,
                tx_ip.c_str(), tx_port, rx_port);
  }

  template <typename T> void sendTlm(TlmMsg<T> msg, uint16_t apid) {
    uint8_t buf[sizeof(T) + 6];
    SpacePacket::encode(buf, &msg.msg, sizeof(msg.msg), PacketType::TELEMETRY,
                        apid, msg.seq);
    sendto(this->tx_fd_, buf, sizeof(buf), 0,
           (struct sockaddr *)&this->tx_addr_, sizeof(this->tx_addr_));
  }

private:
  int tx_fd_;
  int rx_fd_;
  struct sockaddr_in tx_addr_ {};
  struct sockaddr_in rx_addr_ {};

  rclcpp::Logger logger;
  char *prefix;

  std::shared_ptr<std::thread> recv_thread_;
  std::atomic_bool running_;
  RecvCb recv_cb_;
};

} // namespace cdh_link

#endif // CDH_LINK_HPP_