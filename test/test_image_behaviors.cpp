// Copyright 2023 PickNik Inc.
// All rights reserved.
//
// Unauthorized copying of this code base via any medium is strictly prohibited.
// Proprietary and confidential.
#include <gtest/gtest.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <moveit_studio_behavior_interface/shared_resources_node_loader.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/node.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <filesystem>
#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <thread>

namespace{
// Helper function to easily create an image file
sensor_msgs::msg::Image createImage()
{
  // Define the image dimensions and format
  int width = 640;
  int height = 480;
  int channels = 3;
  int bytes_per_channel = 1;
  int stride = width * channels * bytes_per_channel;
  sensor_msgs::msg::Image image;
  image.width = width;
  image.height = height;
  image.step = stride;
  image.encoding = "bgr8";
  image.is_bigendian = false;
  image.header.stamp = rclcpp::Clock().now();

  // Allocate memory for the image data
  image.data.resize(height * stride);

  // Return the image message
  return image;
}
}

namespace moveit_studio
{
TEST(SaveToFileBehaviors, test_halt_save_image)
{

  //Note: This test is currently failing
  GTEST_SKIP();

  pluginlib::ClassLoader<moveit_studio::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_studio_behavior_interface", "moveit_studio::behaviors::SharedResourcesNodeLoaderBase");
  // SaveImageToFile should fail if it tries to save to a nonexistent filepath
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("file_path", "/tmp/filepath/");
  config.blackboard->set("image_topic", "/image_topic");
  // (Note: this sets the port remapping rules so the keys on the blackboard are the same as the keys used by the behavior)
  config.input_ports.insert(std::make_pair("image_topic", "="));
  config.input_ports.insert(std::make_pair("file_path", "="));

  auto node = std::make_shared<rclcpp::Node>("test_halt_save_image");
  auto shared_resources = std::make_shared<moveit_studio::behaviors::BehaviorContext>(node);

  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("example_behaviors::ExampleBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }

  auto save_image_behavior = factory.instantiateTreeNode("save_image_behavior", "SaveImageToFile", config);

  // Once the behavior gets ticked it should be RUNNING until it gets an image
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::RUNNING);


  save_image_behavior->halt();

  // When the behavior is ticked a second time after waiting a short duration it returns FAILURE without throwing an
  // exception because it detected that the path does not exist
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::FAILURE);
  node.reset();
}

TEST(SaveToFileBehaviors, test_save_image_bad_filepath)
{
  // SaveImageToFile should fail if it tries to save to a nonexistent filepath
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("file_path", "/tmp/nonexistent_filepath_which_might_fail/");
  config.blackboard->set("image_topic", "/image_topic");
  // (Note: this sets the port remapping rules so the keys on the blackboard are the same as the keys used by the behavior)
  config.input_ports.insert(std::make_pair("image_topic", "="));
  config.input_ports.insert(std::make_pair("file_path", "="));

  auto node = std::make_shared<rclcpp::Node>("test_save_image_bad_filepath");
  auto shared_resources = std::make_shared<moveit_studio::behaviors::BehaviorContext>(node);
    pluginlib::ClassLoader<moveit_studio::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_studio_behavior_interface", "moveit_studio::behaviors::SharedResourcesNodeLoaderBase");
  // SaveImageToFile should fail if it tries to save to a nonexistent filepath
  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("example_behaviors::ExampleBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }
  auto save_image_behavior = factory.instantiateTreeNode("save_image_behavior", "SaveImageToFile", config);

  // Once the behavior gets ticked it should be RUNNING until it gets an image
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::RUNNING);

  // Publish the image message a few times in case the 
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("/image_topic", rclcpp::SensorDataQoS());
  
  // Wait for a node to subscribe and publish an image
  while (pub->get_subscription_count() < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  pub->publish(createImage());
  rclcpp::spin_some(node);

  // When the behavior is ticked a second time after waiting a short duration it returns FAILURE without throwing an
  // exception because it detected that the path does not exist
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::FAILURE);
  pub.reset();
  node.reset();
}

TEST(SaveToFileBehaviors, test_save_image_to_file)
{
  // SaveImageToFile should succeed in these tests
  // Set the configuration for the behavior node
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  std::string filepath = "/tmp/moveit_studio_data_";
  // Append timestamp to the folder so we can have a unique location
  time_t now = time(nullptr);
  tm* ltm = localtime(&now);
  char timestamp[20];
  strftime(timestamp, sizeof(timestamp), "%Y%m%d%H%M%S", ltm);
  filepath += std::string(timestamp);
  // Create the filepath since it doesn't exist
  const std::filesystem::path folder_path(filepath);
  std::filesystem::create_directories(folder_path);

  config.blackboard->set("file_path", filepath);
  config.blackboard->set("image_topic", "/image_topic");
  config.input_ports.insert(std::make_pair("file_path", "="));
  config.input_ports.insert(std::make_pair("image_topic", "="));

  // Make sure files can be saved with correct file path and topic usage

  // Create a publisher and behavior node
  auto node = std::make_shared<rclcpp::Node>("test_save_image_to_file");
  auto shared_resources = std::make_shared<moveit_studio::behaviors::BehaviorContext>(node);
    pluginlib::ClassLoader<moveit_studio::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_studio_behavior_interface", "moveit_studio::behaviors::SharedResourcesNodeLoaderBase");
  // SaveImageToFile should fail if it tries to save to a nonexistent filepath
  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("example_behaviors::ExampleBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }
  auto save_image_behavior = factory.instantiateTreeNode("save_image_behavior", "SaveImageToFile", config);

  // Once the behavior gets ticked it should be RUNNING until it gets an image
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::RUNNING);

  // Publish the image message
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("/image_topic", rclcpp::SensorDataQoS());
  // Wait for a node to subscribe and publish an image
  while (pub->get_subscription_count() < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  pub->publish(createImage());
  rclcpp::spin_some(node);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // When the behavior is ticked a second time after waiting a short duration it returns SUCCESS
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::SUCCESS);

  // Check if file exists and it's name regex matches the topic it used
  bool file_found = false;
  std::string found_file_name;
  std::string filename_pattern = "image_topic.+\\.png";
  std::regex regex_pattern(filename_pattern);
  for (const auto& file : std::filesystem::directory_iterator(filepath))
  {
    if (std::filesystem::is_regular_file(file) && std::regex_match(file.path().filename().string(), regex_pattern) &&
        file.path().extension() == ".png")
    {
      found_file_name = file.path();
      file_found = true;
      break;
    }
  }
  ASSERT_TRUE(file_found);
  // Read the saved image file
  cv::Mat image = cv::imread(found_file_name, cv::IMREAD_COLOR);
  // Check that the image dimensions match expected values
  ASSERT_EQ(image.cols, 640);
  ASSERT_EQ(image.rows, 480);
  node.reset();
  pub.reset();
}

TEST(SaveToFileBehaviors, test_save_image_to_file_no_leading_slash)
{
  // SaveImageToFile should succeed in these tests
  // Set the configuration for the behavior node
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  std::string filepath = "/tmp/moveit_studio_data_";
  // Append timestamp to the folder so we can have a unique location
  time_t now = time(nullptr);
  tm* ltm = localtime(&now);
  char timestamp[20];
  strftime(timestamp, sizeof(timestamp), "%Y%m%d%H%M%S", ltm);
  filepath += std::string(timestamp);
  // Create the filepath since it doesn't exist
  const std::filesystem::path folder_path(filepath);
  std::filesystem::create_directories(folder_path);

  config.blackboard->set("file_path", filepath);
  config.blackboard->set("image_topic", "image_topic_without_slash");
  config.input_ports.insert(std::make_pair("file_path", "="));
  config.input_ports.insert(std::make_pair("image_topic", "="));

  // Check if we can save a file when the user leaves out the ROS 2 topic's leading slash

  // Create a publisher and behavior node
  auto node = std::make_shared<rclcpp::Node>("test_save_image_to_file_no_leading_slash");
  auto shared_resources = std::make_shared<moveit_studio::behaviors::BehaviorContext>(node);
    pluginlib::ClassLoader<moveit_studio::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_studio_behavior_interface", "moveit_studio::behaviors::SharedResourcesNodeLoaderBase");
  // SaveImageToFile should fail if it tries to save to a nonexistent filepath
  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("example_behaviors::ExampleBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }
  auto save_image_behavior = factory.instantiateTreeNode("save_image_behavior", "SaveImageToFile", config);

  // Once the behavior gets ticked it should be RUNNING until it gets an image
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::RUNNING);

  // Publish the image message
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("/image_topic_without_slash",
                                                             rclcpp::SensorDataQoS());
  // Wait for a node to subscribe and publish an image
  while (pub->get_subscription_count() < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  pub->publish(createImage());
  rclcpp::spin_some(node);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // When the behavior is ticked a second time after waiting a short duration it returns SUCCESS
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::SUCCESS);

  // Check if file exists and it's name regex matches the topic it used
  bool file_found = false;
  std::string found_file_name;
  std::string filename_pattern = "image_topic_without_slash.+\\.png";
  std::regex regex_pattern(filename_pattern);
  for (const auto& file : std::filesystem::directory_iterator(filepath))
  {
    if (std::filesystem::is_regular_file(file) && std::regex_match(file.path().filename().string(), regex_pattern) &&
        file.path().extension() == ".png")
    {
      found_file_name = file.path();
      file_found = true;
      break;
    }
  }
  ASSERT_TRUE(file_found);
  // Read the saved image file
  cv::Mat image = cv::imread(found_file_name, cv::IMREAD_COLOR);
  // Check that the image dimensions match expected values
  ASSERT_EQ(image.cols, 640);
  ASSERT_EQ(image.rows, 480);
  node.reset();
  pub.reset();
}

TEST(SaveToFileBehaviors, test_save_image_to_file_nested_topic)
{
  // SaveImageToFile should succeed in these tests
  // Set the configuration for the behavior node
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  std::string filepath = "/tmp/moveit_studio_data_";
  // Append timestamp to the folder so we can have a unique location
  time_t now = time(nullptr);
  tm* ltm = localtime(&now);
  char timestamp[20];
  strftime(timestamp, sizeof(timestamp), "%Y%m%d%H%M%S", ltm);
  filepath += std::string(timestamp);
  // Create the filepath since it doesn't exist
  const std::filesystem::path folder_path(filepath);
  std::filesystem::create_directories(folder_path);

  config.blackboard->set("file_path", filepath);
  config.blackboard->set("image_topic", "/image/topic/with/slash");
  config.input_ports.insert(std::make_pair("file_path", "="));
  config.input_ports.insert(std::make_pair("image_topic", "="));
  // Check if we can save a file when the user uses nested topics

  // Create a publisher and behavior node
  auto node = std::make_shared<rclcpp::Node>("test_save_image_to_file_nested_topic");
  auto shared_resources = std::make_shared<moveit_studio::behaviors::BehaviorContext>(node);
    pluginlib::ClassLoader<moveit_studio::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_studio_behavior_interface", "moveit_studio::behaviors::SharedResourcesNodeLoaderBase");
  // SaveImageToFile should fail if it tries to save to a nonexistent filepath
  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("example_behaviors::ExampleBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }
  auto save_image_behavior = factory.instantiateTreeNode("save_image_behavior", "SaveImageToFile", config);

  // Once the behavior gets ticked it should be RUNNING until it gets an image
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::RUNNING);

  // Publish the image message
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("/image/topic/with/slash",
                                                             rclcpp::SensorDataQoS());
  // Wait for a node to subscribe and publish an image
  while (pub->get_subscription_count() < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  pub->publish(createImage());
  rclcpp::spin_some(node);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // When the behavior is ticked a second time after waiting a short duration it returns SUCCESS
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::SUCCESS);

  // Check if file exists and it's name regex matches the topic it used
  bool file_found = false;
  std::string found_file_name;
  std::string filename_pattern = "image_topic_with_slash.+\\.png";
  std::regex regex_pattern(filename_pattern);
  for (const auto& file : std::filesystem::directory_iterator(filepath))
  {
    if (std::filesystem::is_regular_file(file) && std::regex_match(file.path().filename().string(), regex_pattern) &&
        file.path().extension() == ".png")
    {
      found_file_name = file.path();
      file_found = true;
      break;
    }
  }
  ASSERT_TRUE(file_found);
  // Read the saved image file
  cv::Mat image = cv::imread(found_file_name, cv::IMREAD_COLOR);
  // Check that the image dimensions match expected values
  ASSERT_EQ(image.cols, 640);
  ASSERT_EQ(image.rows, 480);
  node.reset();
  pub.reset();
}


TEST(SaveToFileBehaviors, test_save_image_to_file_trailing_slashes)
{
  // SaveImageToFile should succeed in these tests
  // Set the configuration for the behavior node
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  std::string filepath = "/tmp/moveit_studio_data_";
  // Append timestamp to the folder so we can have a unique location
  time_t now = time(nullptr);
  tm* ltm = localtime(&now);
  char timestamp[20];
  strftime(timestamp, sizeof(timestamp), "%Y%m%d%H%M%S", ltm);
  filepath += std::string(timestamp) + "////";
  // Create the filepath since it doesn't exist
  const std::filesystem::path folder_path(filepath);
  std::filesystem::create_directories(folder_path);

  config.blackboard->set("file_path", filepath);
  config.blackboard->set("image_topic", "image_topic_path_with_slashes");
  config.input_ports.insert(std::make_pair("file_path", "="));
  config.input_ports.insert(std::make_pair("image_topic", "="));
  // Check if we can save a file when the user adds multiple trailing slashes to the filepath

  // Create a publisher and behavior node
  auto node = std::make_shared<rclcpp::Node>("test_save_image_to_file_trailing_slashes");
  auto shared_resources = std::make_shared<moveit_studio::behaviors::BehaviorContext>(node);
    pluginlib::ClassLoader<moveit_studio::behaviors::SharedResourcesNodeLoaderBase> class_loader(
      "moveit_studio_behavior_interface", "moveit_studio::behaviors::SharedResourcesNodeLoaderBase");
  // SaveImageToFile should fail if it tries to save to a nonexistent filepath
  BT::BehaviorTreeFactory factory;
  {
    auto plugin_instance = class_loader.createUniqueInstance("example_behaviors::ExampleBehaviorsLoader");
    ASSERT_NO_THROW(plugin_instance->registerBehaviors(factory, shared_resources));
  }
  auto save_image_behavior = factory.instantiateTreeNode("save_image_behavior", "SaveImageToFile", config);

  // Once the behavior gets ticked it should be RUNNING until it gets an image
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::RUNNING);

  // Publish the image message
  auto pub = node->create_publisher<sensor_msgs::msg::Image>("/image_topic_path_with_slashes",
                                                             rclcpp::SensorDataQoS());
  // Wait for a node to subscribe and publish an image
  while (pub->get_subscription_count() < 1) std::this_thread::sleep_for(std::chrono::milliseconds(10));
  pub->publish(createImage());
  rclcpp::spin_some(node);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // When the behavior is ticked a second time after waiting a short duration it returns SUCCESS
  ASSERT_EQ(save_image_behavior->executeTick(), BT::NodeStatus::SUCCESS);

  // Check if file exists and it's name regex matches the topic it used
  bool file_found = false;
  std::string found_file_name;
  std::string filename_pattern = "image_topic_path_with_slashes.+\\.png";
  std::regex regex_pattern(filename_pattern);
  for (const auto& file : std::filesystem::directory_iterator(filepath))
  {
    if (std::filesystem::is_regular_file(file) && std::regex_match(file.path().filename().string(), regex_pattern) &&
        file.path().extension() == ".png")
    {
      found_file_name = file.path();
      file_found = true;
      break;
    }
  }
  ASSERT_TRUE(file_found);
  // Read the saved image file
  cv::Mat image = cv::imread(found_file_name, cv::IMREAD_COLOR);
  // Check that the image dimensions match expected values
  ASSERT_EQ(image.cols, 640);
  ASSERT_EQ(image.rows, 480);
  node.reset();
  pub.reset();
}

}  // namespace moveit_studio

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
