#include "async_printaction_node.hpp"

#include <chrono>
#include <random>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/controls/parallel_node.h"
#include "rclcpp/rclcpp.hpp"

AsyncPrintAction::AsyncPrintAction(const std::string& name,
                                   const BT::NodeConfiguration& config)
    : AsyncActionNode(name, config) {
  rng_.seed(std::random_device()());
  dist_ = std::uniform_real_distribution<double>(0.5, 3.0);
}

BT::PortsList AsyncPrintAction::providedPorts() {
  return {BT::InputPort<std::string>("message"),
          BT::InputPort<int>("robot_id")};
}

BT::NodeStatus AsyncPrintAction::tick() {
  std::string msg;
  int robot_id;
  if (!getInput("message", msg) || !getInput("robot_id", robot_id)) {
    return BT::NodeStatus::FAILURE;
  }
  double delay = dist_(rng_);
  RCLCPP_INFO(rclcpp::get_logger("async_bt_demo"),
              "Robot %d: Starting async action (will take %.1fs)", robot_id,
              delay);
  auto start_time = std::chrono::steady_clock::now();
  auto target_time = start_time + std::chrono::duration<double>(delay);
  while (std::chrono::steady_clock::now() < target_time) {
    if (isHaltRequested()) {
      RCLCPP_INFO(rclcpp::get_logger("async_bt_demo"),
                  "Robot %d: Action cancelled!", robot_id);
      return BT::NodeStatus::IDLE;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  if (isHaltRequested()) {
    RCLCPP_INFO(rclcpp::get_logger("async_bt_demo"),
                "Robot %d: Action cancelled after completion!", robot_id);
    return BT::NodeStatus::IDLE;
  }
  RCLCPP_INFO(rclcpp::get_logger("async_bt_demo"),
              "Robot %d: %s (completed after %.1fs)", robot_id, msg.c_str(),
              delay);
  return BT::NodeStatus::SUCCESS;
}
