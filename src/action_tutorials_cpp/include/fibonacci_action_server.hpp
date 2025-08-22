#include <functional>
#include <memory>
#include <thread>

#include "action_tutorials_cpp/visibility_control.h"
#include "action_tutorials_interfaces_mine/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp {

class FibonacciActionServer : public rclcpp::Node {
 public:
  ACTION_TUTORIALS_CPP_PUBLIC
  explicit FibonacciActionServer(const rclcpp::NodeOptions&);

 private:
  rclcpp_action::Server<action_tutorials_interfaces_mine::action::Fibonacci>::
      SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<
          const action_tutorials_interfaces_mine::action::Fibonacci::Goal>);

  rclcpp_action::CancelResponse handle_cancel(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          action_tutorials_interfaces_mine::action::Fibonacci>>);

  void handle_accepted(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<
          action_tutorials_interfaces_mine::action::Fibonacci>>);

  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<
                   action_tutorials_interfaces_mine::action::Fibonacci>>);
};

}  // namespace action_tutorials_cpp
