cmake_minimum_required(VERSION 3.0.2)
project(occupancy_gridmap_core)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  grid_map_core
  grid_map_cv
  nav_msgs  
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    grid_map_core
    grid_map_cv
    nav_msgs
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
  src/OccupancyGridMap.cpp
  src/OccupancyGridMapHelper.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)