cmake_minimum_required(VERSION 3.0.2)
project(wavefront_planner_demos)
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
    # wavefront_planner_demos
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

# Wavefront Demo node
add_executable(wavefront_planner_demos 
  src/wavefront_planner_demos_node.cpp
  src/WavefrontSearchDemo.cpp # Wavefront search demo
  src/CostmapDemo.cpp # Potential field demo
  src/WavePropagatorDemo.cpp
)
target_link_libraries(wavefront_planner_demos
  ${catkin_LIBRARIES}
)