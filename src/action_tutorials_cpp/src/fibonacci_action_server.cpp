#include "fibonacci_action_server.hpp"

#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_cpp/visibility_control.h"
#include "action_tutorials_interfaces_mine/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

action_tutorials_cpp::FibonacciActionServer::FibonacciActionServer(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("fibonacci_action_server", options) {
  this->action_server_ = rclcpp_action::create_server<
      action_tutorials_interfaces_mine::action::Fibonacci>(
      this, "fibonacci",
      std::bind(&FibonacciActionServer::handle_goal, this,
                std::placeholders::_1, std::placeholders::_2),
      std::bind(&FibonacciActionServer::handle_cancel, this,
                std::placeholders::_1),
      std::bind(&FibonacciActionServer::handle_accepted, this,
                std::placeholders::_1));
}

rclcpp_action::GoalResponse
action_tutorials_cpp::FibonacciActionServer::handle_goal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<
        const action_tutorials_interfaces_mine::action::Fibonacci::Goal>
        goal) {
  RCLCPP_INFO(this->get_logger(), "Received goal request with order %d",
              goal->order);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
action_tutorials_cpp::FibonacciActionServer::handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        action_tutorials_interfaces_mine::action::Fibonacci>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void action_tutorials_cpp::FibonacciActionServer::handle_accepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        action_tutorials_interfaces_mine::action::Fibonacci>>
        goal_handle) {
  // this needs to return quickly to avoid blocking the executor, so spin up a
  // new thread
  std::thread{
      std::bind(&FibonacciActionServer::execute, this, std::placeholders::_1),
      goal_handle}
      .detach();
}

void action_tutorials_cpp::FibonacciActionServer::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<
        action_tutorials_interfaces_mine::action::Fibonacci>>
        goal_handle) {
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const std::shared_ptr<
      const action_tutorials_interfaces_mine::action::Fibonacci_Goal>
      goal = goal_handle->get_goal();
  std::shared_ptr<action_tutorials_interfaces_mine::action::Fibonacci_Feedback>
      feedback = std::make_shared<
          action_tutorials_interfaces_mine::action::Fibonacci::Feedback>();
  std::vector<int32_t, std::allocator<int32_t>>& sequence =
      feedback->partial_sequence;
  sequence.push_back(0);
  sequence.push_back(1);
  std::shared_ptr<action_tutorials_interfaces_mine::action::Fibonacci_Result>
      result = std::make_shared<
          action_tutorials_interfaces_mine::action::Fibonacci::Result>();
  for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->sequence = sequence;
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    // Update sequence
    sequence.push_back(sequence[i] + sequence[i - 1]);
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(this->get_logger(), "Publish feedback");
    loop_rate.sleep();
  }
  // Check if goal is done
  if (rclcpp::ok()) {
    result->sequence = sequence;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

RCLCPP_COMPONENTS_REGISTER_NODE(action_tutorials_cpp::FibonacciActionServer)
