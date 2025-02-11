cmake_minimum_required(VERSION 3.0.2)
project(layered_exercise)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  turtlesim
  geometry_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(
  DIRECTORY launch 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
