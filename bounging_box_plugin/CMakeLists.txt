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

# Build the plugin as a shared library
add_library(bounding_box_plugin SHARED src/bounding_box_plugin.cpp)

# Include directories
target_include_directories(bounding_box_plugin PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Specify the C++ standard
target_compile_features(bounding_box_plugin PUBLIC c_std_99 cxx_std_17)

# Link against ROS 2 and Gazebo ROS libraries
ament_target_dependencies(
  bounding_box_plugin
  rclcpp
  geometry_msgs
  gazebo_ros
  visualization_msgs
  tf2_ros
)

# Link Gazebo libraries (if required for your plugin)
target_link_libraries(bounding_box_plugin
  ${GAZEBO_LIBRARIES}
)

# Install the shared library
install(TARGETS bounding_box_plugin
  LIBRARY DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # The following line skips the linter which checks for copyrights
  # comment the line when a copyright and license are added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # The following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license are added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
