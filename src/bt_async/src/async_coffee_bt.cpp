#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include "nodes/brew_coffee_node.hpp"
#include "nodes/grind_beans_node.hpp"
#include "nodes/heat_water_node.hpp"
#include "nodes/say_something_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;
  // Register all nodes
  factory.registerNodeType<HeatWater>("HeatWater");
  factory.registerNodeType<GrindBeans>("GrindBeans");
  factory.registerNodeType<BrewCoffee>("BrewCoffee");
  factory.registerNodeType<SaySomething>("SaySomething");
  // Load behavior tree
  std::string package_share_dir =
      ament_index_cpp::get_package_share_directory("bt_async");
  std::string bt_file = package_share_dir + "/bt/async_coffee.xml";
  BT::Tree tree = factory.createTreeFromFile(bt_file);
  // Add loggers
  BT::StdCoutLogger logger_cout(tree);
  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Starting Async Coffee Maker Behavior Tree!");
  RCLCPP_INFO(rclcpp::get_logger("main"), "Each step will take random time...");
  // We need to spin because async nodes create their own ROS nodes
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  // Tick the tree in a loop
  while (rclcpp::ok()) {
    BT::NodeStatus status = tree.tickRoot();
    if (status == BT::NodeStatus::SUCCESS) {
      RCLCPP_INFO(rclcpp::get_logger("main"),
                  "Behavior Tree completed successfully!");
      break;
    } else if (status == BT::NodeStatus::FAILURE) {
      RCLCPP_ERROR(rclcpp::get_logger("main"), "Behavior Tree failed!");
      break;
    }
    // Process ROS callbacks
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  rclcpp::shutdown();
  return 0;
}
