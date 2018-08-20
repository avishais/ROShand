cmake_minimum_required(VERSION 2.8.3)
project(planner)

# set(CMAKE_CXX_FLAGS "-std=c++11")
# add_definitions(-std=c++11)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

find_package(OMPL)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

add_service_files(
  FILES
  plan_req.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES planner
  CATKIN_DEPENDS roscpp 
#  DEPENDS system_lib
)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/include
  include/planner_node
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLLUDE_DIR}
)

#add_library(${PROJECT_NAME} SHARED  
#  src/plan.cpp
#  src/myRRT.cpp
#)

add_executable(${PROJECT_NAME}_node src/plan.cpp src/myRRT.cpp src/planner_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${OMPL_LIBRARIES}
  ${EIGEN3_LIBS}
  ${Boost_LIBRARIES}
)

# add_dependencies(${PROJECT_NAME}_node)


