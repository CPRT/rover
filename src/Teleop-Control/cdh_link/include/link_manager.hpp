#ifndef LINK_MANAGER_HPP_
#define LINK_MANAGER_HPP_

#include <stdint.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"

#include "cdh_link.hpp"

namespace cdh_link {

class VideoStream {
public:
  VideoStream(rclcpp::Node *parent, const std::string &node_prefix,
              int32_t bitrate = 1000000, LinkId link = L2400);

  void setLink(Link link);
  void setBitrate(int32_t bitrate);

  LinkId getLinkId() { return this->link_id_; };
  int32_t getBitrate() { return this->bitrate_; };

private:
  rclcpp::Node *parent_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr ip_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr bitrate_pub_;

  LinkId link_id_;
  int32_t bitrate_;
};

class LinkManager {
public:
  LinkManager(rclcpp::Node *parent, Link link_900, Link link_2400);

  void process();

private:
  rclcpp::Node *parent_;

  Link link_900_;
  Link link_2400_;

  LinkStatus status_900_;
  LinkStatus status_2400_;

  VideoStream rtp_;
  VideoStream mosaic_;
};

}; // namespace cdh_link

#endif // LINK_MANAGER_HPP_
