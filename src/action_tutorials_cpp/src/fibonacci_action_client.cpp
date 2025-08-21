#include "fibonacci_action_client.hpp"

#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "action_tutorials_interfaces/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

action_tutorials_cpp::FibonacciActionClient::FibonacciActionClient(
    const rclcpp::NodeOptions& options)
    : Node("fibonacci_action_client", options) {
  this->client_ptr_ = rclcpp_action::create_client<
      action_tutorials_interfaces::action::Fibonacci>(this, "fibonacci");
  this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&FibonacciActionClient::send_goal, this));
}

void action_tutorials_cpp::FibonacciActionClient::send_goal() {
  this->timer_->cancel();
  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Action server not available after waiting");
    rclcpp::shutdown();
  }
  action_tutorials_interfaces::action::Fibonacci::Goal goal_msg =
      action_tutorials_interfaces::action::Fibonacci::Goal();
  goal_msg.order = 10;
  RCLCPP_INFO(this->get_logger(), "Sending goal");
  rclcpp_action::Client<action_tutorials_interfaces::action::Fibonacci>::
      SendGoalOptions send_goal_options = rclcpp_action::Client<
          action_tutorials_interfaces::action::Fibonacci>::SendGoalOptions();
  send_goal_options.goal_response_callback =
      std::bind(&FibonacciActionClient::goal_response_callback, this,
                std::placeholders::_1);
  send_goal_options.feedback_callback =
      std::bind(&FibonacciActionClient::feedback_callback, this,
                std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(
      &FibonacciActionClient::result_callback, this, std::placeholders::_1);
  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void action_tutorials_cpp::FibonacciActionClient::goal_response_callback(
    const rclcpp_action::ClientGoalHandle<
        action_tutorials_interfaces::action::Fibonacci>::SharedPtr&
        goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

void action_tutorials_cpp::FibonacciActionClient::feedback_callback(
    rclcpp_action::ClientGoalHandle<
        action_tutorials_interfaces::action::Fibonacci>::SharedPtr,
    const std::shared_ptr<
        const action_tutorials_interfaces::action::Fibonacci::Feedback>
        feedback) {
  std::stringstream ss;
  ss << "Next number in sequence received: ";
  for (int number : feedback->partial_sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
}

void action_tutorials_cpp::FibonacciActionClient::result_callback(
    const rclcpp_action::ClientGoalHandle<
        action_tutorials_interfaces::action::Fibonacci>::WrappedResult&
        result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  std::stringstream ss;
  ss << "Result received: ";
  for (int number : result.result->sequence) {
    ss << number << " ";
  }
  RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  rclcpp::shutdown();
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionClient)
