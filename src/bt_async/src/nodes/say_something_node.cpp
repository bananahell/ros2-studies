#include "nodes/say_something_node.hpp"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "rclcpp/rclcpp.hpp"

SaySomething::SaySomething(const std::string& name,
                           const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

BT::PortsList SaySomething::providedPorts() {
  return {BT::InputPort<std::string>("message")};
}

BT::NodeStatus SaySomething::tick() {
  std::string message;
  if (getInput("message", message)) {
    RCLCPP_INFO(rclcpp::get_logger("SaySomething"), "%s", message.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}
