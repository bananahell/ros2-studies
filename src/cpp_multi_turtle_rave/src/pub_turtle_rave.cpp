#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// chrono bind make_shared
using namespace std;
// Twist
using namespace geometry_msgs::msg;
// Node TimerBase Publisher init spin shutdown
using namespace rclcpp;

class TurtleRavePub : public Node {
 public:
  const char* pub1topic = "/turtlesim1/turtle1/cmd_vel";

  TurtleRavePub() : Node("turtle_rave_pub") {
    publisher1_ = this->create_publisher<Twist>(pub1topic, 10);
    timer_ = this->create_wall_timer(
        chrono::milliseconds(500), bind(&TurtleRavePub::timer_callback, this));
  }

 private:
  TimerBase::SharedPtr timer_;

  Publisher<Twist>::SharedPtr publisher1_;

  void timer_callback() {
    auto message = Twist();
    message.linear.x = 2;
    message.angular.z = 1.8;
    publisher1_->publish(message);
  }
};

int main(int argc, char* argv[]) {
  init(argc, argv);
  spin(make_shared<TurtleRavePub>());
  shutdown();
  return 0;
}
