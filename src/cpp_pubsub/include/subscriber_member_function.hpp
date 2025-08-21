#ifndef SUBSCRIBER_MEMBER_FUNCTION
#define SUBSCRIBER_MEMBER_FUNCTION

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber();

 private:
  void topic_callback(const std_msgs::msg::String&) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

#endif  // SUBSCRIBER_MEMBER_FUNCTION
