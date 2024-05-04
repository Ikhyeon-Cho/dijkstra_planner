cmake_minimum_required(VERSION 3.0.2)
project(wavefront_core)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_cv
  grid_map_ros
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    grid_map_core
    grid_map_cv
    grid_map_ros
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
  src/costmap/CostMap.cpp
  src/costmap/CostMapConverter.cpp
  src/occupancy_grid/OccupancyMap.cpp
  src/occupancy_grid/OccupancyMapHelper.cpp
  src/occupancy_grid/OccupancyMapConverter.cpp
  src/wavefront_expansion/WavePropagator.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)