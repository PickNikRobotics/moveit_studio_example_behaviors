find_package(ament_cmake_gtest REQUIRED)

ament_add_gtest(
  test_behavior_plugins
  test_behavior_plugins.cpp
)
ament_target_dependencies(test_behavior_plugins ${THIS_PACKAGE_INCLUDE_DEPENDS})

ament_add_gtest(
  test_image_behaviors
  test_image_behaviors.cpp)

ament_target_dependencies(test_image_behaviors ${THIS_PACKAGE_INCLUDE_DEPENDS})
