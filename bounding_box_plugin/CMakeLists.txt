cmake_minimum_required(VERSION 3.8)
project(bounding_box_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(bounding_box_msg REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

# Build the plugin as a shared library
add_library(bounding_box_plugin_lib SHARED src/bounding_box_plugin.cpp)

# Include directories
target_include_directories(bounding_box_plugin_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_BINARY_DIR}/rosidl_generator_cpp>  # Add this line for generated headers
  $<INSTALL_INTERFACE:include>
)

ament_export_dependencies(rosidl_default_runtime std_msgs std_srvs)

# Link against ROS 2 and Gazebo ROS libraries
ament_target_dependencies(
  bounding_box_plugin_lib
  rclcpp
  geometry_msgs
  gazebo_ros
  visualization_msgs
  tf2_ros
  bounding_box_msg
  pcl_ros
  pcl_conversions
  )

# Link Gazebo libraries (if required for your plugin)
target_link_libraries(bounding_box_plugin_lib
  ${GAZEBO_LIBRARIES}
)

# Install the shared library
install(TARGETS bounding_box_plugin_lib
  LIBRARY DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
