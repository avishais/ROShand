cmake_minimum_required(VERSION 2.8.3)
project(action_node)
  
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_msgs_gl
  message_generation
)

generate_messages()

catkin_package(
)

include_directories(
  include/action_node
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(action_node src/action_node.cpp src/action.cpp)

target_link_libraries(action_node
  ${catkin_LIBRARIES}
)

add_dependencies(action_node common_msgs_gl_gencpp)


