#include "nodes/heat_water_node.hpp"

#include "behaviortree_cpp_v3/bt_factory.h"
#include "nodes/async_action_node.hpp"

HeatWater::HeatWater(const std::string& name,
                     const BT::NodeConfiguration& config)
    : AsyncAction(name, config) {}

std::string HeatWater::getActionName() const { return "Heating water"; }

int HeatWater::getMinDuration() const { return 2000; }

int HeatWater::getMaxDuration() const { return 5000; }

BT::PortsList HeatWater::providedPorts() { return {}; }
