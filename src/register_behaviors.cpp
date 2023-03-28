#include <behaviortree_cpp_v3/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <hello_world/hello_world.hpp>
#include <hello_world/delayed_message.hpp>
#include <hello_world/setup_mtc_wave_hand.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace hello_world
{
class HelloWorldBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<HelloWorld>(factory, "HelloWorld", shared_resources);
    moveit_studio::behaviors::registerBehavior<DelayedMessage>(factory, "DelayedMessage", shared_resources);
    moveit_studio::behaviors::registerBehavior<SetupMTCWaveHand>(factory, "SetupMTCWaveHand", shared_resources);

  }
};
}  // namespace hello_world

PLUGINLIB_EXPORT_CLASS(hello_world::HelloWorldBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
