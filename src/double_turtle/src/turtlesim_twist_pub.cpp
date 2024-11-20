#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// bind make_shared
using namespace std;
// milliseconds
using namespace std::chrono;
// Node TimerBase init spin shutdown
using namespace rclcpp;
// Twist
using namespace geometry_msgs::msg;

class TurtlesimTwistPub : public Node {
 public:
  TurtlesimTwistPub() : Node("turtlesim_twist_pub") {
    const string pubName = "/turtlesim1/turtle1/cmd_vel";
    publisher_ = this->create_publisher<Twist>(pubName, 10);
    timer_ = this->create_wall_timer(
        milliseconds(500), bind(&TurtlesimTwistPub::timer_callback, this));
  }

 private:
  void timer_callback() {
    auto message = Twist();
    message.linear.x = 3;
    message.angular.z = 1;
    publisher_->publish(message);
  }

  TimerBase::SharedPtr timer_;

  Publisher<Twist>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
  init(argc, argv);
  spin(make_shared<TurtlesimTwistPub>());
  shutdown();
  return 0;
}
