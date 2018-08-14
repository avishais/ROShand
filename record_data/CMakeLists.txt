cmake_minimum_required(VERSION 2.8.3)
project(record_data)
  
set(CMAKE_CXX_FLAGS "-std=c++11")
find_package(catkin REQUIRED COMPONENTS
  roscpp
  common_msgs_gl
  message_generation
  cv_bridge
)

find_package(OpenCV REQUIRED)

generate_messages()

catkin_package(
)

include_directories(
  include/record_data
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(record_data src/record_data.cpp src/main_record_data.cpp)
target_link_libraries(record_data ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(record_data common_msgs_gl_gencpp)

add_executable(play_bags src/play_bags.cpp)
target_link_libraries(play_bags ${catkin_LIBRARIES})
