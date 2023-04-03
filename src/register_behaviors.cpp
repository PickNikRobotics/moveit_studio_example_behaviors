#include <behaviortree_cpp_v3/bt_factory.h>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>

#include <example_behaviors/hello_world.hpp>
#include <example_behaviors/delayed_message.hpp>
#include <example_behaviors/setup_mtc_wave_hand.hpp>
#include <image_behaviors/save_image_to_file.hpp>
#include <image_behaviors/image_to_blackboard.hpp>
#include <image_behaviors/blackboard_image_to_file.hpp>

#include <pluginlib/class_list_macros.hpp>

namespace example_behaviors
{
class ExampleBehaviorsLoader : public moveit_studio::behaviors::SharedResourcesNodeLoaderBase
{
public:
  void registerBehaviors(BT::BehaviorTreeFactory& factory,
                     const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources) override
  {
    moveit_studio::behaviors::registerBehavior<HelloWorld>(factory, "HelloWorld", shared_resources);
    moveit_studio::behaviors::registerBehavior<DelayedMessage>(factory, "DelayedMessage", shared_resources);
    moveit_studio::behaviors::registerBehavior<SetupMTCWaveHand>(factory, "SetupMTCWaveHand", shared_resources);
    moveit_studio::behaviors::registerBehavior<image_behaviors::SaveImageToFile>(factory, "SaveImageToFile", shared_resources);
    moveit_studio::behaviors::registerBehavior<image_behaviors::ImageToBlackboard>(factory, "ImageToBlackboard", shared_resources);
    moveit_studio::behaviors::registerBehavior<image_behaviors::BlackboardImageToFile>(factory, "BlackboardImageToFile", shared_resources);

  }
};
}  // namespace example_behaviors

PLUGINLIB_EXPORT_CLASS(example_behaviors::ExampleBehaviorsLoader,
                       moveit_studio::behaviors::SharedResourcesNodeLoaderBase);
