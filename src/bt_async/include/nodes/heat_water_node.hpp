#pragma once

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nodes/async_action_node.hpp"

class HeatWater : public AsyncAction {
 public:
  HeatWater(const std::string&, const BT::NodeConfiguration&);

  std::string getActionName() const override;

  int getMinDuration() const override;

  int getMaxDuration() const override;

  static BT::PortsList providedPorts();
};
