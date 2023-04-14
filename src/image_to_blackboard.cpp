#include <image_behaviors/image_to_blackboard.hpp>

namespace
{
constexpr auto kPortIDInputTopic = "input_topic";
constexpr auto kPortIDOutputName = "output_port";
const auto callbackTimeout = 5000;
}  // namespace
namespace image_behaviors
{

ImageToBlackboard::ImageToBlackboard(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList ImageToBlackboard::providedPorts()
{
  return { 
    BT::InputPort<std::string>(kPortIDInputTopic),
    BT::OutputPort<sensor_msgs::msg::Image::SharedPtr>(kPortIDOutputName),
  };
}

fp::Result<void> ImageToBlackboard::doHalt()
{      
  RCLCPP_WARN(rclcpp::get_logger(name()), "ImageToBlackboard behavior halted");
  return {};
}

fp::Result<bool> ImageToBlackboard::doWork()
{
  const auto image_topic = getInput<std::string>(kPortIDInputTopic);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(image_topic); error)
  {
    return tl::make_unexpected(fp::Internal("Missing input port: " + error.value()));
  }

// Create our image topic subscriber
  rclcpp::CallbackGroup::SharedPtr cb_group_waitset =
      shared_resources_->node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
  auto subscription_options = rclcpp::SubscriptionOptions();
  subscription_options.callback_group = cb_group_waitset;
  auto subscription_callback = [this](const sensor_msgs::msg::Image::SharedPtr msg) { subscriberCallback(msg); };

  subscription_ = shared_resources_->node->create_subscription<sensor_msgs::msg::Image>(
      image_topic.value(), rclcpp::QoS(1).best_effort().keep_last(1), subscription_callback, subscription_options);
  wait_set_.add_subscription(subscription_);

  // Wait for the subscriber event to trigger using a timeout.
  const auto wait_result = wait_set_.wait(std::chrono::milliseconds(callbackTimeout));
  switch (wait_result.kind())
  {
    case rclcpp::WaitResultKind::Ready:
    {
      // If a message arrives on the topic, set the blackboard output using the subscriber callback
      sensor_msgs::msg::Image msg;
      rclcpp::MessageInfo msg_info;
      if (subscription_->take(msg, msg_info))
      {
        std::shared_ptr<void> type_erased_msg = std::make_shared<sensor_msgs::msg::Image>(msg);
        subscription_->handle_message(type_erased_msg, msg_info);
      }
      return true;
    }
    case rclcpp::WaitResultKind::Timeout:
      return tl::make_unexpected(fp::Internal("Timeout. No message received after " + std::to_string(callbackTimeout / 1000) + " seconds"));
    default:
      return tl::make_unexpected(fp::Internal("Error. Wait-set failed."));
  }
  subscription_.reset();

  // Return SUCCESS if the callback SUCCEEDED
  return (true);
}

void ImageToBlackboard::subscriberCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Write to blackboard
  setOutput(kPortIDOutputName, msg);
}
}  // namespace image_behaviors
