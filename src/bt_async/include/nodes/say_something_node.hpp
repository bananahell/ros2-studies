#pragma once

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "rclcpp/rclcpp.hpp"

class SaySomething : public BT::SyncActionNode {
 public:
  SaySomething(const std::string&, const BT::NodeConfiguration&);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};
