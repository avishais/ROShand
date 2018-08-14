cmake_minimum_required(VERSION 2.8.3)
project(gripper_state_publisher)

set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(catkin REQUIRED COMPONENTS
  openhand
  roscpp
)

catkin_package(
)

include_directories(
  include/gripper_state_publisher
  ${catkin_INCLUDE_DIRS}
)


add_executable(gripper_state_publisher_node src/gripper_state_publisher.cpp src/node.cpp)

add_dependencies(gripper_state_publisher_node openhand_gencpp)
target_link_libraries(gripper_state_publisher_node
  ${catkin_LIBRARIES}
)
