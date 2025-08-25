#pragma once

#include <chrono>
#include <random>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/controls/parallel_node.h"
#include "rclcpp/rclcpp.hpp"

class AsyncPrintAction : public BT::AsyncActionNode {
 public:
  AsyncPrintAction(const std::string&, const BT::NodeConfiguration&);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;

 private:
  std::mt19937 rng_;

  std::uniform_real_distribution<double> dist_;
};
