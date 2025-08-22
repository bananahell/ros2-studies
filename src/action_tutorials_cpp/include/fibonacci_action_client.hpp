#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>

#include "action_tutorials_interfaces_mine/action/fibonacci.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace action_tutorials_cpp {

class FibonacciActionClient : public rclcpp::Node {
 public:
  explicit FibonacciActionClient(const rclcpp::NodeOptions&);

  void send_goal();

 private:
  rclcpp_action::Client<action_tutorials_interfaces_mine::action::Fibonacci>::
      SharedPtr client_ptr_;

  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(
      const rclcpp_action::ClientGoalHandle<
          action_tutorials_interfaces_mine::action::Fibonacci>::SharedPtr&);

  void feedback_callback(
      rclcpp_action::ClientGoalHandle<
          action_tutorials_interfaces_mine::action::Fibonacci>::SharedPtr,
      const std::shared_ptr<
          const action_tutorials_interfaces_mine::action::Fibonacci::Feedback>);

  void result_callback(
      const rclcpp_action::ClientGoalHandle<
          action_tutorials_interfaces_mine::action::Fibonacci>::WrappedResult&);
};

}  // namespace action_tutorials_cpp
