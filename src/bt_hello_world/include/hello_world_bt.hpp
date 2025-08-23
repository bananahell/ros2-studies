#ifndef HELLO_WORLD_BT
#define HELLO_WORLD_BT

#include "behaviortree_cpp_v3/bt_factory.h"

class SaySomething : public BT::SyncActionNode {
 public:
  SaySomething(const std::string&, const BT::NodeConfiguration&);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

#endif
