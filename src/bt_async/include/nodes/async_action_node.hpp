#pragma once

#include <chrono>
#include <random>
#include <thread>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

class AsyncAction : public BT::StatefulActionNode {
 protected:
  rclcpp::Node::SharedPtr node_;

  std::thread worker_thread_;

  bool finished_ = false;

  BT::NodeStatus result_ = BT::NodeStatus::FAILURE;

  std::random_device rd_;

  std::mt19937 gen_;

 public:
  AsyncAction(const std::string&, const BT::NodeConfiguration&);

  virtual ~AsyncAction();

  virtual std::string getActionName() const = 0;

  virtual int getMinDuration() const = 0;

  virtual int getMaxDuration() const = 0;

  BT::NodeStatus onStart() override;

  BT::NodeStatus onRunning() override;

  void onHalted() override;
};
