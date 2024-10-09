cmake_minimum_required(VERSION 3.5)
project(air_sim)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(urdf REQUIRED)
find_package(xacro REQUIRED)
find_package(gazebo_ros REQUIRED)

# Python script installation (odom_tf_broadcaster.py)
install(PROGRAMS
  scripts/odom_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch, models, config, and worlds directories
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY models
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config files
install(FILES
  config/air.rviz
  config/pcd.rviz
  config/odom.rviz
  DESTINATION share/${PROJECT_NAME}/config
)

# Export dependencies
ament_export_dependencies(rclcpp rviz2 tf2 tf2_ros urdf xacro gazebo_ros)

ament_package()
