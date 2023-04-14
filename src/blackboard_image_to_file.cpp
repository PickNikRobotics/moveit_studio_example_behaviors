#include <image_behaviors/blackboard_image_to_file.hpp>

namespace
{
constexpr auto kPortIDOutputPath = "file_path";
constexpr auto kPortIDOutputName = "file_name";
constexpr auto kPortIDInputImage = "input_image";
}  // namespace


namespace image_behaviors
{

BlackboardImageToFile::BlackboardImageToFile(const std::string& name, const BT::NodeConfiguration& config,
                                 const std::shared_ptr<moveit_studio::behaviors::BehaviorContext>& shared_resources)
  : moveit_studio::behaviors::AsyncBehaviorBase(name, config, shared_resources)
{
}

BT::PortsList BlackboardImageToFile::providedPorts()
{
  return { 
    BT::InputPort<std::string>(kPortIDOutputPath),
    BT::InputPort<std::string>(kPortIDOutputName),
    BT::InputPort<sensor_msgs::msg::Image::SharedPtr>(kPortIDInputImage),
  };
}

fp::Result<bool> BlackboardImageToFile::doWork()
{
  const auto image_message = getInput<sensor_msgs::msg::Image::SharedPtr>(kPortIDInputImage);
  const auto filepath = getInput<std::string>(kPortIDOutputPath);
  const auto image_name = getInput<std::string>(kPortIDOutputName);

  // Check that all required input data ports were set
  if (const auto error = moveit_studio::behaviors::maybe_error(image_message, filepath, image_name); error)
  {
    return tl::make_unexpected(fp::Internal("Missing input port: " + error.value()));
  }

  // Assemble the file path for saving the image
  std::filesystem::path path(filepath.value());

  // Add current timestamp to filename
  time_t now = time(nullptr);
  tm* ltm = localtime(&now);
  char timestamp[21];
  strftime(timestamp, sizeof(timestamp), "%Y%m%d_%H%M%S", ltm);
  const auto filename = path.string() + std::filesystem::path::preferred_separator + image_name.value() + "_" + std::string(timestamp) + ".png";

  // Convert the image message to an OpenCV image and write to disk
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(image_message.value(), sensor_msgs::image_encodings::BGR8);
    if (cv::imwrite(filename, cv_ptr->image))
    {
      RCLCPP_INFO(rclcpp::get_logger(name()), "Saved image to file: %s", filename.c_str());
      return true;
    }
    else
    {
      // If cv:imwrite fails without throwing an exception
      return tl::make_unexpected(fp::Internal("Error writing image: " + filename));
    }
  }
  catch (cv::Exception& e)
  {
    return tl::make_unexpected(fp::Internal(std::string(e.what()) + " Error writing image: " + filename));
  }

}
}  // namespace image_behaviors


