// Copyright 2023 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.
#include <moveit_studio_behavior_interface/async_behavior_base.hpp>

#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <opencv2/imgcodecs.hpp>
#include <moveit_studio_behavior_interface/behavior_context.hpp>
#include <moveit_studio_behavior_interface/shared_resources_node.hpp>
#include <moveit_studio_behavior_interface/check_for_error.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace image_behaviors
{
/**
 * @brief Save an image from the blackboard to disk.
 *
 * @details
 * | Data Port Name | Port Type | Object Type                          |
 * | -------------- | --------- | ------------------------------------ |
 * | file_path      | input     | std::string                          |
 * | file_name      | input     | std::string                          |
 * | input_image    | input     | sensor_msgs::msg::Image::SharedPtr   |
 */
class BlackboardImageToFile final : public moveit_studio::behaviors::AsyncBehaviorBase
{
public:
  BlackboardImageToFile(const std::string& name, const BT::NodeConfiguration& config,
                  const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources);

  static BT::PortsList providedPorts();

private:
  /**
   * @brief Saves an image from the black to the location given on an input port, using the name given on an input port
   * @return Returns void if the subscriber executed successfully. Returns a failure result if unsuccessful.
   */
  fp::Result<bool> doWork() override;

  /** @brief Classes derived from AsyncBehaviorBase must implement getFuture() so that it returns a shared_future class member */
  std::shared_future<fp::Result<bool>>& getFuture() override
  {
    return future_;
  }

  /** @brief Classes derived from AsyncBehaviorBase must have this shared_future as a class member */
  std::shared_future<fp::Result<bool>> future_;
};
}  // namespace image_behaviors
