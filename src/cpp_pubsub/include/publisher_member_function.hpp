#ifndef PUBLISHER_MEMBER_FUNCTION
#define PUBLISHER_MEMBER_FUNCTION

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalPublisher : public rclcpp::Node {
 public:
  MinimalPublisher();

 private:
  void timer_callback();

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  size_t count_;
};

#endif  // PUBLISHER_MEMBER_FUNCTION
