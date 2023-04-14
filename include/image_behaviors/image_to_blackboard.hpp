// Copyright 2023 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>

#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <condition_variable>
#include <mutex>
#include <thread>

namespace image_behaviors
{
/**
 * @brief Move an image from a topic to the blackboard.
 *
 * @details
 * | Data Port Name | Port Type | Object Type                         |
 * | -------------- | --------- | ----------------------------------- |
 * | input_topic    | input     | std::string                         |
 * | output_port    | output    | sensor_msgs::msg::Image::SharedPtr  |
 */
class ImageToBlackboard final : public moveit_studio::behaviors::AsyncBehaviorBase
{
public:
  ImageToBlackboard(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

private:
  /**
   * @brief Wait for an image on the specified topic, then put it on the blackboard
   * @return Returns an error message if unsuccessful.
   */
  fp::Result<bool> doWork() override;
  /**
   * @brief Additional actions to perform if halted
   */
  fp::Result<void> doHalt() override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<fp::Result<bool>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<fp::Result<bool>> future_;
  /**
   * @brief The image subscriber callback.
   * @param msg Image message received.
   */
  void subscriberCallback(const sensor_msgs::msg::Image::SharedPtr msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::WaitSet wait_set_;

};
}  // namespace image_behaviors
