cmake_minimum_required(VERSION 3.8)
project(image_conversion_pkg)

# C++17 Standard
set(CMAKE_CXX_STANDARD 17)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(image_transport REQUIRED)

# Create executable
add_executable(image_conversion_node 
  src/image_conversion_node.cpp
)

# Link dependencies
ament_target_dependencies(image_conversion_node
  rclcpp
  sensor_msgs
  std_srvs
  cv_bridge
  OpenCV
  image_transport
)

# Install executable
install(TARGETS
  image_conversion_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
