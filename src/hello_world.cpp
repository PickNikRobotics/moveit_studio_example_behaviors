#include <example_behaviors/hello_world.hpp>

namespace example_behaviors
{
HelloWorld::HelloWorld(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::SharedResourcesNode<BT::SyncActionNode>(name, config, shared_resources)
{
}


BT::PortsList HelloWorld::providedPorts()
{
  // This node has no input or output ports
  return BT::PortsList({});
}

BT::NodeStatus HelloWorld::tick()
{
  // Do HelloWorld's useful work
  shared_resources_->failure_logger->publishFailureMessage(
    name(),
    MoveItStudioErrorCode{ moveit_msgs::msg::MoveItErrorCodes::SUCCESS, "Hello, world!" }, "");

  // Nodes that return FAILURE will have their error messages displayed in the UI.
  // If this node returns SUCCESS, the failure message will only be printed in the logs
  return BT::NodeStatus::SUCCESS;
}

}  // namespace example_behaviors
