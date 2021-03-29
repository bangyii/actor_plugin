cmake_minimum_required(VERSION 2.8.3)
project(actor_plugin)

set(RVO_HEADERS
	src/rvo/RVO.h
	src/rvo/RVOSimulator.h
	src/rvo/Vector2.h)

set(RVO_SOURCES
	src/rvo/Agent.cpp
	src/rvo/Agent.h
	src/rvo/Definitions.h
	src/rvo/KdTree.cpp
	src/rvo/KdTree.h
	src/rvo/Obstacle.cpp
	src/rvo/Obstacle.h
	src/rvo/RVOSimulator.cpp)

# Check for c++11 / c++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  geometry_msgs
  nav_msgs
  pedsim_msgs
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

# find_package(ignition-common1 QUIET REQUIRED COMPONENTS profiler)
find_package(ignition-math4 REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

add_library(${PROJECT_NAME} src/actor_plugin.cc)
add_library(RVO ${RVO_HEADERS} ${RVO_SOURCES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ignition-math4::ignition-math4 RVO)

add_executable(rvo_simulation src/rvo_simulation.cpp)
target_link_libraries(rvo_simulation ${catkin_LIBRARIES} RVO)