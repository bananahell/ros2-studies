#include "say_something_node.hpp"

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
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("SaySomething"),
                 "Missing required input [message]");
    return BT::NodeStatus::FAILURE;
  }
}
