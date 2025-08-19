#include "publisher_member_function.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

MinimalPublisher::MinimalPublisher() : Node("minimal_publisher"), count_(0) {
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&MinimalPublisher::timer_callback, this));
}

void MinimalPublisher::timer_callback() {
  std_msgs::msg::String message = std_msgs::msg::String();
  message.data = "Hello, world! " + std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
