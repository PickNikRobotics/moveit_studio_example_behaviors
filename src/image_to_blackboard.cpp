#include <image_behaviors/image_to_blackboard.hpp>

namespace
{
constexpr auto kPortIDInputTopic = "input_topic";
constexpr auto kPortIDOutputName = "output_port";
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

fp::Result<bool> ImageToBlackboard::doWork()
{
  const auto image_topic = getInput<std::string>(kPortIDInputTopic);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(image_topic); error)
  {
    return tl::make_unexpected(fp::Internal("Missing input port: " + error.value()));
  }

// Create our image topic subscriber
  auto options = rclcpp::SubscriptionOptions();
  image_subscriber_ = shared_resources_->node->create_subscription<sensor_msgs::msg::Image>(
      image_topic.value(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) { subscriberCallback(msg); }, options);

  // Wait until the callback has completed
  std::unique_lock<std::mutex> lock(mutex_);
  // while (!callback_success_)
  // {
    condition_var_.wait(lock);
  // }
  // Kill the subscriber
  image_subscriber_.reset();

  // Return SUCCESS if the callback SUCCEEDED
  return (true);
}

void ImageToBlackboard::subscriberCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Write to blackboard
  setOutput(kPortIDOutputName, msg);
  condition_var_.notify_all();
}
}  // namespace image_behaviors
