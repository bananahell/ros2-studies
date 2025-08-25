#include <chrono>
#include <random>

#include "async_printaction_node.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/controls/parallel_node.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;
  // Register our async node
  factory.registerNodeType<AsyncPrintAction>("AsyncPrint");
  // Create the behavior tree
  std::string xml_text = R"(
    <root BTCPP_format="4">
        <BehaviorTree>
            <Parallel success_threshold="3" failure_threshold="1">
                <AsyncPrint message="Hello from Robot 1!" robot_id="1"/>
                <AsyncPrint message="Hello from Robot 2!" robot_id="2"/>
                <AsyncPrint message="Hello from Robot 3!" robot_id="3"/>
            </Parallel>
        </BehaviorTree>
    </root>
    )";
  auto tree = factory.createTreeFromText(xml_text);
  RCLCPP_INFO(rclcpp::get_logger("async_bt_demo"),
              "Starting async behavior tree demo...");
  // Run the tree - CORRECTED METHOD
  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  while (status == BT::NodeStatus::RUNNING) {
    status = tree.tickRoot();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_INFO(rclcpp::get_logger("async_bt_demo"),
              "Demo completed with status: %s",
              status == BT::NodeStatus::SUCCESS   ? "SUCCESS"
              : status == BT::NodeStatus::FAILURE ? "FAILURE"
                                                  : "IDLE");
  rclcpp::shutdown();
  return 0;
}
