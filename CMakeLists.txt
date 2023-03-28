cmake_minimum_required(VERSION 3.16.3)
project(hello_world)

# Set MoveIt's typical CMake settings, including C++ standard version
find_package(moveit_common REQUIRED)
moveit_package()

find_package(moveit_studio_behavior_interface REQUIRED)
find_package(pluginlib REQUIRED)

set(
  THIS_PACKAGE_INCLUDE_DEPENDS
  moveit_studio_behavior_interface
  pluginlib
)

add_library(
  hello_world SHARED
  src/hello_world.cpp
  src/delayed_message.cpp
  src/setup_mtc_wave_hand.cpp
  src/register_behaviors.cpp
)
target_include_directories(
  hello_world
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  PUBLIC $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(hello_world ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install Libraries
install(
  TARGETS hello_world
  EXPORT hello_worldTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  add_subdirectory(test)
  ament_lint_auto_find_test_dependencies()
endif()

# Export the behavior plugins defined in this package so they are available to plugin loaders that load the
# behavior base class library from the moveit_studio_behavior package.
pluginlib_export_plugin_description_file(moveit_studio_behavior_interface hello_world_plugin_description.xml)

ament_export_targets(hello_worldTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
