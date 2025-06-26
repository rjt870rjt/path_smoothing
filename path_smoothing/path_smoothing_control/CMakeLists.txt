cmake_minimum_required(VERSION 3.8)
project(path_smoothing_control)

# Use C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Node executables
add_executable(waypoint_publisher src/waypoint_publisher.cpp)
add_executable(path_smoother src/path_smoother.cpp)
add_executable(trajectory_generator src/trajectory_generator.cpp)
add_executable(trajectory_follower src/trajectory_follower.cpp)
add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
add_executable(metrics_node src/metrics_node.cpp)

# Link libraries
ament_target_dependencies(waypoint_publisher rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(path_smoother rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(trajectory_generator rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(trajectory_follower
  rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
ament_target_dependencies(obstacle_avoidance rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(metrics_node rclcpp geometry_msgs nav_msgs)
ament_target_dependencies(trajectory_follower rclcpp geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)


# Install targets
install(TARGETS
  waypoint_publisher
  path_smoother
  trajectory_generator
  trajectory_follower
  obstacle_avoidance
  metrics_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install RViz config
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()
