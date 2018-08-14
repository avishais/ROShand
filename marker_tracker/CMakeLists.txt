cmake_minimum_required(VERSION 2.8.3)
project(marker_tracker)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  image_transport
  cv_bridge
  message_generation
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  ImageSpacePoseMsg.msg
#   Message2.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ImageSpacePoseSrv.srv
#   Service2.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES marker_tracker
  CATKIN_DEPENDS roscpp roslib image_transport cv_bridge message_runtime
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/marker_tracker.cpp
# )

# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(marker_generator src/marker_generator.cpp src/main_marker_generator.cpp)
add_executable(marker_tracker src/marker_tracker.cpp src/main_marker_tracker.cpp)

# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

add_dependencies(marker_tracker marker_tracker_gencpp)

target_link_libraries(marker_generator
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
target_link_libraries(marker_tracker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
