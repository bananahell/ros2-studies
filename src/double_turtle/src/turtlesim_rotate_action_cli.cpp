#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/action/rotate_absolute.hpp"

// stringstream bind make_shared
using namespace std;
// milliseconds
using namespace std::chrono;
// _1 _2
using namespace std::placeholders;
// Node TimerBase init spin shutdown
using namespace rclcpp;
// ClientGoalHandle ResultCode
using namespace rclcpp_action;
// RotateAbsolute
using namespace turtlesim::action;
// Twist
using namespace geometry_msgs::msg;

class TurtlesimRotateActionCli : public Node {
 public:
  TurtlesimRotateActionCli() : Node("turtlesim_rotate_action_cli") {
    const string cliName = "/turtlesim2/turtle1/rotate_absolute";
    const string pubName = "/turtlesim2/turtle1/cmd_vel";
    client_ = rclcpp_action::create_client<RotateAbsolute>(this, cliName);
    publisher_ = this->create_publisher<Twist>(pubName, 10);
    currAngle = 0;
    timer_ = this->create_wall_timer(
        milliseconds(1000), bind(&TurtlesimRotateActionCli::send_goal, this));
  }

 private:
  TimerBase::SharedPtr timer_;

  rclcpp_action::Client<RotateAbsolute>::SharedPtr client_;

  Publisher<Twist>::SharedPtr publisher_;

  float currAngle;

  void send_goal() {
    this->timer_->cancel();

    if (!this->client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = RotateAbsolute::Goal();
    currAngle += M_PI / 2;
    goal_msg.theta = currAngle;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<RotateAbsolute>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        bind(&TurtlesimRotateActionCli::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        bind(&TurtlesimRotateActionCli::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        bind(&TurtlesimRotateActionCli::result_callback, this, _1);
    this->client_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(
      const ClientGoalHandle<RotateAbsolute>::SharedPtr& goal_handle) {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(),
                  "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
      ClientGoalHandle<RotateAbsolute>::SharedPtr,
      const shared_ptr<const RotateAbsolute::Feedback> feedback) {
    stringstream ss;
    ss << "Next number in sequence received: ";
    ss << feedback->remaining;
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(
      const ClientGoalHandle<RotateAbsolute>::WrappedResult& result) {
    switch (result.code) {
      case ResultCode::SUCCEEDED:
        break;
      case ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    stringstream ss;
    ss << "Result received: ";
    ss << result.result->delta << " ";
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    timer_callback();
  }

  void timer_callback() {
    auto message = Twist();
    message.linear.x = 3;
    publisher_->publish(message);
    timer_ = this->create_wall_timer(
        milliseconds(500), bind(&TurtlesimRotateActionCli::send_goal, this));
  }
};

int main(int argc, char* argv[]) {
  init(argc, argv);
  spin(make_shared<TurtlesimRotateActionCli>());
  shutdown();
  return 0;
}
