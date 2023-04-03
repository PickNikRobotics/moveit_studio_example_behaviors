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
 * @brief Save an image from a topic to disk.
 *
 * @details
 * | Data Port Name | Port Type | Object Type                   |
 * | -------------- | --------- | ----------------------------- |
 * | file_path       | input     | std::string                   |
 * | image_topic    | input     | std::string                   |
 */
class ImageToBlackboard final : public moveit_studio::behaviors::AsyncBehaviorBase
{
public:
  ImageToBlackboard(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

private:
  /**
   * @brief Saves an image from the specified topic to the location given on the input port
   * @return Returns void if the subscriber executed successfully. Returns a failure result if the subscriber could not
   * be created.
   */
  fp::Result<bool> doWork() override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<fp::Result<bool>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<fp::Result<bool>> future_;
  std::mutex mutex_;
  std::condition_variable condition_var_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  /**
   * @brief The image subscriber callback.
   * @param msg Subscriber message received.
   */
  void subscriberCallback(const sensor_msgs::msg::Image::SharedPtr msg);

};
}  // namespace image_behaviors
