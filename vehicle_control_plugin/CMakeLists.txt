cmake_minimum_required(VERSION 3.8)
project(vehicle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(gazebo_ros)
find_package(gazebo_dev)
find_package(gazebo_msgs)
find_package(nav_msgs)
find_package(geometry_msgs)
find_package(ackermann_msgs REQUIRED)
find_package(tf2_ros)

link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(vehicle_control SHARED
  src/vehicle_control.cpp
)

ament_target_dependencies(vehicle_control
  rclcpp
  gazebo_ros
  nav_msgs
  geometry_msgs
  ackermann_msgs
  tf2_ros
)

target_include_directories(vehicle_control PUBLIC 
  include
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(vehicle_control
  ${GAZEBO_LIBRARIES}
)

install(TARGETS 
  vehicle_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  PROGRAMS
  scripts/teleop_keyboard.py
  scripts/sd_msgs_to_ackermann.py
  scripts/sd_teleop_keyboard_control.py
  scripts/sd_teleop_with_vehicle_interface.py
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
