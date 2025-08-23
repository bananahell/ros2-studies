#ifndef SAY_SOMETHING_NODE
#define SAY_SOMETHING_NODE

#include "behaviortree_cpp_v3/action_node.h"

class SaySomething : public BT::SyncActionNode {
 public:
  SaySomething(const std::string&, const BT::NodeConfiguration&);

  static BT::PortsList providedPorts();

  BT::NodeStatus tick() override;
};

#endif
