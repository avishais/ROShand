cmake_minimum_required(VERSION 2.8.3)
project(move_gripper)
  
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(catkin REQUIRED COMPONENTS
  openhand
  roscpp
  common_msgs_gl
  message_generation
  dynamixel_motors
)

add_service_files(
  FILES
  CommandTrajectory.srv
)

generate_messages()

catkin_package(
)

include_directories(
  include/
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(move_gripper src/move_gripper_node.cpp src/move_gripper.cpp)
add_executable(dynamixel_node1 src/main_dynamixel_node.cpp)

target_link_libraries(move_gripper
  ${catkin_LIBRARIES}
)
target_link_libraries(dynamixel_node1
  ${catkin_LIBRARIES}
)

add_dependencies(move_gripper openhand_gencpp common_msgs_gl_gencpp)

