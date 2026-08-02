#ifndef NODE_STATUS_PUBLISHER_HPP
#define NODE_STATUS_PUBLISHER_HPP

#include <interfaces/msg/node_list.hpp>
#include <interfaces/srv/get_topics.hpp>
#include <rclcpp/rclcpp.hpp>

#include <string>
#include <vector>

class NodeStatusPublisher : public rclcpp::Node {
public:
  NodeStatusPublisher();

private:
  void publish_nodes();

  void
  get_topics(const std::shared_ptr<interfaces::srv::GetTopics::Request> request,
             std::shared_ptr<interfaces::srv::GetTopics::Response> response);

  double frequency_;

  rclcpp::Publisher<interfaces::msg::NodeList>::SharedPtr publisher_;
  rclcpp::Service<interfaces::srv::GetTopics>::SharedPtr topics_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

#endif // NODE_STATUS_PUBLISHER_HPP