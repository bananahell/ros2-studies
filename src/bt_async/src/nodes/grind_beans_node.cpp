#include "nodes/grind_beans_node.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nodes/async_action_node.hpp"

GrindBeans::GrindBeans(const std::string& name,
                       const BT::NodeConfiguration& config)
    : AsyncAction(name, config) {}

std::string GrindBeans::getActionName() const { return "Grinding beans"; }

int GrindBeans::getMinDuration() const { return 1000; }

int GrindBeans::getMaxDuration() const { return 3000; }

BT::PortsList GrindBeans::providedPorts() { return {}; }
