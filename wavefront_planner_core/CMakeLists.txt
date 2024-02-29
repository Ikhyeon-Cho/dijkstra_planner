cmake_minimum_required(VERSION 3.0.2)
project(wavefront_planner_core)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  occupancy_gridmap_core
  costmap_core
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    grid_map_core
    occupancy_gridmap_core
    costmap_core
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/WavePropagator.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)