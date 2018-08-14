cmake_minimum_required(VERSION 2.8.3)
project(test_node)
  
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
  include/test_nodes
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_node src/test_node.cpp src/test.cpp)

target_link_libraries(test_node
  ${catkin_LIBRARIES}
)

add_dependencies(test_node common_msgs_gl_gencpp)


