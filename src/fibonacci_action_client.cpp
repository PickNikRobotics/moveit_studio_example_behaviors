#include <example_behaviors/fibonacci_action_client.hpp>

// Include the template implementation for ActionClientBehaviorBase<T>.
#include <moveit_studio_behavior_interface/impl/action_client_behavior_base_impl.hpp>

namespace example_behaviors
{
FibonacciActionClient::FibonacciActionClient(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : ActionClientBehaviorBase<Fibonacci>(name, config, shared_resources)
{
}

BT::PortsList FibonacciActionClient::providedPorts()
{
  // This node has two input ports and two output ports
  return BT::PortsList({
      BT::InputPort<std::string>("action_name", "/fibonacci", "The name of the action to send a goal to."),
      BT::InputPort<std::size_t>("order", "The order of the Fibonacci sequence"),
      BT::OutputPort<std::vector<int>>("feedback", "{feedback}",
                                       "The action feedback containing the latest element in the current Fibonacci "
                                       "sequence."),
      BT::OutputPort<std::vector<int>>(
          "result", "{result}", "The result containing the entire Fibonacci sequence up to the specified order."),
  });
}

BT::KeyValueVector FibonacciActionClient::metadata()
{
  return { { "subcategory", "Example" },
           { "description", "Calls the Fibonacci action and makes the result available on an output port." } };
}

tl::expected<std::string, std::string> FibonacciActionClient::getActionName()
{
  const auto action_name = getInput<std::string>("action_name");
  if (const auto error = moveit_studio::behaviors::maybe_error(action_name))
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }
  return action_name.value();
}

tl::expected<Fibonacci::Goal, std::string> FibonacciActionClient::createGoal()
{
  const auto order = getInput<std::size_t>("order");

  if (const auto error = moveit_studio::behaviors::maybe_error(order))
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }

  return example_interfaces::build<Fibonacci::Goal>().order(order.value());
}

tl::expected<bool, std::string> FibonacciActionClient::processResult(const std::shared_ptr<Fibonacci::Result> result)
{
  std::stringstream stream;
  for (const auto& value : result->sequence)
  {
    stream << value << " ";
  }
  std::cout << stream.str() << std::endl;

  // Publish the result to the UI
  shared_resources_->logger->publishInfoMessage(name(), "Result: " + stream.str(), false);

  setOutput<std::vector<int>>("result", result->sequence);

  return { true };
}

void FibonacciActionClient::processFeedback(const std::shared_ptr<const Fibonacci::Feedback> feedback)
{
  std::stringstream stream;
  for (const auto& value : feedback->sequence)
  {
    stream << value << " ";
  }
  std::cout << stream.str() << std::endl;

  // Publish the feedback to the UI
  shared_resources_->logger->publishInfoMessage(name(), "Feedback: " + stream.str(), false);

  setOutput<std::vector<int>>("feedback", feedback->sequence);
}
}  // namespace example_behaviors
