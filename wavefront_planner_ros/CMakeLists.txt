cmake_minimum_required(VERSION 3.0.2)
project(wavefront_planner)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  wavefront_core
  wavefront_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # wavefront_planner
  CATKIN_DEPENDS 
    roscpp
    tf2_ros
    wavefront_core
    wavefront_msgs
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wavefront_planner_node 
  src/wavefront_planner_node.cpp
  src/WavefrontPlanner.cpp
)

target_link_libraries(wavefront_planner_node
  ${catkin_LIBRARIES}
)