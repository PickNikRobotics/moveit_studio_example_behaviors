#include <hello_world/hello_world.hpp>

namespace hello_world
{
HelloWorld::HelloWorld(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}


BT::PortsList HelloWorld::providedPorts()
{
  // TODO(...)
  return BT::PortsList({});
}

BT::NodeStatus HelloWorld::tick()
{
  // TODO(...)
  // Return SUCCESS once the work has been completed.
  return BT::NodeStatus::SUCCESS;
}

}  // namespace hello_world
