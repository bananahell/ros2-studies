#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "turtlesim/action/rotate_absolute.hpp"

// stringstream bind make_shared
using namespace std;
// milliseconds
using namespace std::chrono;
// _1 _2
using namespace std::placeholders;
// RotateAbsolute
using namespace turtlesim::action;
// Node TimerBase init spin shutdown
using namespace rclcpp;
// ClientGoalHandle ResultCode
using namespace rclcpp_action;

class TurtlesimRotateActionCli : public Node {
 public:
  TurtlesimRotateActionCli() : Node("turtlesim_rotate_action_cli") {
    const string cliName = "/turtlesim2/turtle1/rotate_absolute";
    client_ = rclcpp_action::create_client<RotateAbsolute>(this, cliName);
    currAngle = 0;
    timer_ = this->create_wall_timer(
        milliseconds(5000), bind(&TurtlesimRotateActionCli::send_goal, this));
  }

 private:
  TimerBase::SharedPtr timer_;

  rclcpp_action::Client<RotateAbsolute>::SharedPtr client_;

  float currAngle;

  void send_goal() {
    if (!this->client_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = RotateAbsolute::Goal();
    currAngle++;
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
  }
};

int main(int argc, char* argv[]) {
  init(argc, argv);
  spin(make_shared<TurtlesimRotateActionCli>());
  shutdown();
  return 0;
}
