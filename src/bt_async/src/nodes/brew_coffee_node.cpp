#include "nodes/brew_coffee_node.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nodes/async_action_node.hpp"

BrewCoffee::BrewCoffee(const std::string& name,
                       const BT::NodeConfiguration& config)
    : AsyncAction(name, config) {}

std::string BrewCoffee::getActionName() const { return "Brewing coffee"; }

int BrewCoffee::getMinDuration() const { return 3000; }

int BrewCoffee::getMaxDuration() const { return 6000; }

BT::PortsList BrewCoffee::providedPorts() { return {}; }
