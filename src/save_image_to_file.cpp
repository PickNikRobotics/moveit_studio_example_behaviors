// Copyright 2023 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.

#include <image_behaviors/save_image_to_file.hpp>

namespace
{
constexpr auto kPortIDOutputName = "file_path";
constexpr auto kPortIDImage = "image_topic";
}  // namespace

namespace image_behaviors
{

// If there are leading slashes, remove them and replace all other slashes with underscores
std::string removeSlashes(std::string str)
{
  if (!str.empty() && str.front() == '/')
  {
    // Remove leading slash
    str = str.substr(1);
    // Replace remaining slashes
    std::replace(str.begin(), str.end(), '/', '_');
  }
  return str;
}

SaveImageToFile::SaveImageToFile(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList SaveImageToFile::providedPorts()
{
  return {
    BT::InputPort<std::string>(kPortIDOutputName),
    BT::InputPort<std::string>(kPortIDImage),
  };
}

fp::Result<void> SaveImageToFile::doHalt()
{
  condition_var_.notify_all();  // notify all waiting threads
  return {};
}

fp::Result<bool> SaveImageToFile::doWork()
{

  // Set our initial callback status
  callback_success_ = false;
  const auto image_topic = getInput<std::string>(kPortIDImage);
  const auto filepath = getInput<std::string>(kPortIDOutputName);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(image_topic, filepath); error)
  {
    return tl::make_unexpected(fp::Internal("Missing input port: " + error.value()));
  }

  std::filesystem::path path(filepath.value());
  path = path / removeSlashes(image_topic.value());

  // Add the topic to the filename
  filename_ += path.string() + "_";
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
  return (callback_success_);
}

void SaveImageToFile::subscriberCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Acquire mutex for callback_complete flag
  std::lock_guard<std::mutex> lock(mutex_);
  // Convert the image message to an OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    // Assemble the filename with current timestamp:
    time_t now = time(nullptr);
    tm* ltm = localtime(&now);
    char timestamp[21];
    strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", ltm);
    // Output filename example: /tmp/moveit_studio_data/wrist_mounted_camera_image_raw_YYYYMMDD_HHMMSS.png
    filename_ += std::string(timestamp) + ".png";

    // Write the image to disk
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    if (cv::imwrite(filename_, cv_ptr->image))
    {
      RCLCPP_INFO(rclcpp::get_logger(name()), "Saved image to file: %s", filename_.c_str());
      callback_success_ = true;
    }
    else
    {
      // cv:imwrite can fail without throwing an exception
      shared_resources_->failure_logger->publishFailureMessage(
          name(),
          MoveItStudioErrorCode{ moveit_studio_agent_msgs::msg::MotionPlanStatus::FAILURE, "Error writing image: " },
          std::strerror(errno));
      callback_success_ = false;
    }
  }
  catch (cv::Exception& e)
  {
    shared_resources_->failure_logger->publishFailureMessage(
        name(),
        MoveItStudioErrorCode{ moveit_studio_agent_msgs::msg::MotionPlanStatus::FAILURE, "Image write exception: " },
        e.what());
    callback_success_ = false;
  }

  condition_var_.notify_all();  // notify all waiting threads
}
}  // namespace image_behaviors
