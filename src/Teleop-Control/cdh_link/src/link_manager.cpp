#include "link_manager.hpp"

namespace cdh_link {

VideoStream::VideoStream(rclcpp::Node *parent, const std::string &node_prefix,
                         int32_t bitrate, LinkId link_id)
    : parent_(parent), link_id_(link_id), bitrate_(bitrate) {
  this->ip_pub_ = this->parent_->create_publisher<std_msgs::msg::String>(
      node_prefix + "/set_dest_ip", 1);
  this->bitrate_pub_ = this->parent_->create_publisher<std_msgs::msg::Int32>(
      node_prefix + "/set_bitrate", 1);
}

void VideoStream::setBitrate(int32_t bitrate) {
  std_msgs::msg::Int32 msg;
  msg.data = bitrate;
  if (this->bitrate_pub_) {
    this->bitrate_pub_->publish(msg);
    this->bitrate_ = bitrate;
  }
}

void VideoStream::setLink(Link link) {
  std_msgs::msg::String msg;
  msg.data = link.ground_ip;
  if (this->ip_pub_) {
    this->ip_pub_->publish(msg);
    this->link_id_ = link.id;
  }
}

LinkManager::LinkManager(rclcpp::Node *parent, Link link_900, Link link_2400)
    : parent_(parent), link_900_(link_900), link_2400_(link_2400),
      rtp_(parent, "/rtp_node", 500000), mosaic_(parent, "/mosaic_node") {}

void LinkManager::process() {
  if ((!status_2400_.alive && rtp_.getLinkId() == L2400) || (!status_900_.alive && rtp_.getLinkId() == L900)) {
    rtp_.setLink(rtp_.getLinkId() == L900 ? link_2400_ : link_900_);
  }


}

} // namespace cdh_link
