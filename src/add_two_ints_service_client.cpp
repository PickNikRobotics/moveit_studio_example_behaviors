#include <example_behaviors/add_two_ints_service_client.hpp>

// Include the template implementation for GetMessageFromTopicBehaviorBase<T>.
#include <moveit_studio_behavior_interface/impl/service_client_behavior_base_impl.hpp>
namespace example_behaviors
{
AddTwoIntsServiceClient::AddTwoIntsServiceClient(
    const std::string& name, const BT::NodeConfiguration& config,
    const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : ServiceClientBehaviorBase<example_interfaces::srv::AddTwoInts>(name, config, shared_resources)
{
}

BT::PortsList AddTwoIntsServiceClient::providedPorts()
{
  // This node has three input ports and one output port
  return BT::PortsList({
      BT::InputPort<std::string>("service_name"),
      BT::InputPort<int>("addend1"),
      BT::InputPort<int>("addend2"),
      BT::OutputPort<int>("result"),
  });
}

tl::expected<std::string, std::string> AddTwoIntsServiceClient::getServiceName()
{
  const auto service_name = getInput<std::string>("service_name");
  if (const auto error = moveit_studio::behaviors::maybe_error(service_name))
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }
  return service_name.value();
}

tl::expected<AddTwoInts::Request, std::string> AddTwoIntsServiceClient::createRequest()
{
  const auto a = getInput<int>("addend1");
  const auto b = getInput<int>("addend2");
  if (const auto error = moveit_studio::behaviors::maybe_error(a, b))
  {
    return tl::make_unexpected("Failed to get required value from input data port: " + error.value());
  }
  return example_interfaces::build<AddTwoInts::Request>().a(a.value()).b(b.value());
}

tl::expected<bool, std::string> AddTwoIntsServiceClient::processResponse(const AddTwoInts::Response& response)
{
  setOutput<int>("result", response.sum);
  return { true };
}
}  // namespace example_behaviors
