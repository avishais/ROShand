cmake_minimum_required(VERSION 2.8.3)
project(play_actions_node)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  common_msgs_gl
  message_generation
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(play_actions_node src/play_actions_node.cpp src/play_actions.cpp)

target_link_libraries(play_actions_node
  ${catkin_LIBRARIES}
)

add_dependencies(play_actions_node common_msgs_gl_gencpp)



