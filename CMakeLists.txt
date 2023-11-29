cmake_minimum_required(VERSION 3.0.2)
project(wavefront_planner)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  occupancy_grid_map
  isr_ros_utils
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # wavefront_planner
  CATKIN_DEPENDS 
    roscpp
    occupancy_grid_map
    isr_ros_utils
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wavefront_planner_node src/global_planner_node.cpp
  src/global_planner/GlobalPlanner.cpp
  src/wavefront_planner/WavefrontPlanner.cpp
  src/wavefront_planner/WavefrontPlannerRosConverter.cpp
  
)
target_link_libraries(wavefront_planner_node
  ${catkin_LIBRARIES}
)