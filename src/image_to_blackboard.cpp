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

  auto options = rclcpp::SubscriptionOptions();
  image_subscriber_ = shared_resources_->node->create_subscription<sensor_msgs::msg::Image>(
      image_topic.value(), rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Image::SharedPtr msg) {}, options);

  rclcpp::WaitSet wait_set_;
  // Wait until the callback has completed
  wait_set_.add_subscription(image_subscriber_);

  // Wait for the subscriber event to trigger. Set a 1000 ms margin to trigger a timeout.
  const auto wait_result = wait_set_.wait(std::chrono::milliseconds(1000));
  switch (wait_result.kind())
  {
    case rclcpp::WaitResultKind::Ready:
    {
      sensor_msgs::msg::Image msg;
      rclcpp::MessageInfo msg_info;
      if (image_subscriber_->take(msg, msg_info))
      {  
        // Write to blackboard
        setOutput(kPortIDOutputName, std::make_shared<sensor_msgs::msg::Image>(msg));
        return true;
      }
      else
      {
        RCLCPP_ERROR(rclcpp::get_logger(name()), "Error. Failed to get message.");
        return false;
      }
    }
    case rclcpp::WaitResultKind::Timeout:
      RCLCPP_WARN(rclcpp::get_logger(name()), "Timeout. No message received after given wait-time");
      return false;
    default:
      RCLCPP_ERROR(rclcpp::get_logger(name()), "Error. Wait-set failed.");
      return false;
  }
}
}