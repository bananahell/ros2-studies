#include "nodes/async_action_node.hpp"

#include <chrono>
#include <random>
#include <thread>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

AsyncAction::AsyncAction(const std::string& name,
                         const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config), gen_(rd_()) {
  node_ = std::make_shared<rclcpp::Node>("async_node_" + name);
}

AsyncAction::~AsyncAction() {
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

BT::NodeStatus AsyncAction::onStart() {
  finished_ = false;
  // Generate random duration
  std::uniform_int_distribution<> distr(getMinDuration(), getMaxDuration());
  int duration_ms = distr(gen_);
  RCLCPP_INFO(node_->get_logger(), "%s started (will take %d ms)",
              getActionName().c_str(), duration_ms);
  // Start async work in separate thread
  worker_thread_ = std::thread([this, duration_ms]() {
    std::this_thread::sleep_for(std::chrono::milliseconds(duration_ms));
    result_ = BT::NodeStatus::SUCCESS;
    finished_ = true;
    RCLCPP_INFO(node_->get_logger(), "%s completed", getActionName().c_str());
  });
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus AsyncAction::onRunning() {
  if (finished_) {
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
    return result_;
  }
  return BT::NodeStatus::RUNNING;
}

void AsyncAction::onHalted() {
  RCLCPP_WARN(node_->get_logger(), "%s was halted", getActionName().c_str());
  finished_ = true;
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}
