#include <example_behaviors/get_string_from_topic.hpp>

// Include the template implementation for GetMessageFromTopicBehaviorBase<T>.
#include <moveit_studio_behavior_interface/impl/get_message_from_topic_impl.hpp>

namespace example_behaviors
{
GetStringFromTopic::GetStringFromTopic(const std::string& name, const BT::NodeConfiguration& config,
                    const std::shared_ptr<BehaviorContext>& shared_resources)
  : GetMessageFromTopicBehaviorBase<std_msgs::msg::String>(name, config, shared_resources)
{
}
}  // namespace example_behaviors

// Template specialization of GetMessageFromTopicBehaviorBase<T> for the String message type
template class moveit_studio::behaviors::GetMessageFromTopicBehaviorBase<std_msgs::msg::String>;
