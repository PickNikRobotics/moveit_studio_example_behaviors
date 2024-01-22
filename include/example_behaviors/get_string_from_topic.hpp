#pragma once

#include <moveit_studio_behavior_interface/get_message_from_topic.hpp>
#include <std_msgs/msg/string.hpp>

using moveit_studio::behaviors::BehaviorContext;
using moveit_studio::behaviors::GetMessageFromTopicBehaviorBase;

namespace example_behaviors
{
class GetStringFromTopic final : public GetMessageFromTopicBehaviorBase<std_msgs::msg::String>
{
public:
  GetStringFromTopic(const std::string& name, const BT::NodeConfiguration& config,
            const std::shared_ptr<BehaviorContext>& shared_resources);

private:
  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<tl::expected<bool, std::string>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<tl::expected<bool, std::string>> future_;
};
}  // namespace example_behaviors
