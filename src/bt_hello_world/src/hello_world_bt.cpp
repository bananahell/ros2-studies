#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "rclcpp/rclcpp.hpp"
#include "say_something_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<SaySomething>("SaySomething");
  std::string package_share_dir =
      ament_index_cpp::get_package_share_directory("bt_hello_world");
  std::string bt_file = package_share_dir + "/bt/hello_world.xml";
  BT::Tree tree = factory.createTreeFromFile(bt_file);
  BT::StdCoutLogger logger_cout(tree);
  RCLCPP_INFO(rclcpp::get_logger("main"),
              "Starting Behavior Tree Hello World!");
  tree.tickRoot();
  RCLCPP_INFO(rclcpp::get_logger("main"), "Behavior Tree execution completed!");
  rclcpp::shutdown();
  return 0;
}
