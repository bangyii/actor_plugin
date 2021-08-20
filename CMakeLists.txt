cmake_minimum_required(VERSION 2.8.3)
project(actor_plugin)

# Check for c++17 
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_FLAGS "-std=c++17")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++17 support. Please use a different C++ compiler.")
endif()

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  gazebo_ros
  geometry_msgs
  nav_msgs
  pedsim_msgs
  tf2
  gamma_simulator
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

#find_package(ignition-math4 REQUIRED)

#Noetic find_package
find_package(ignition-math6 REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  DEPENDS
    roscpp
    gazebo_ros
)

add_library(${PROJECT_NAME} src/actor_plugin.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ignition-math6::ignition-math6)

add_library(attach_collision_plugin SHARED src/attach_collision_plugin.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})