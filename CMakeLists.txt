cmake_minimum_required(VERSION 3.5)
project(rosp_layered_exercise)

# Find ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories (ensure headers are found)
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${turtlesim_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
)

# Add executables
add_executable(layered_exercise_node src/move_turtle.cpp)
ament_target_dependencies(layered_exercise_node rclcpp turtlesim geometry_msgs)

add_executable(kill_turtle_node src/kill_turtle.cpp)
ament_target_dependencies(kill_turtle_node rclcpp turtlesim)

add_executable(spawn_turtle_node src/spawn_turtle.cpp)
ament_target_dependencies(spawn_turtle_node rclcpp turtlesim)

# Install executables
install(TARGETS layered_exercise_node kill_turtle_node spawn_turtle_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()