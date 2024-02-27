cmake_minimum_required(VERSION 3.0.2)
project(occupancy_gridmap_msgs)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  grid_map_msgs
  grid_map_ros
  occupancy_gridmap_core
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    grid_map_msgs
    grid_map_ros
    occupancy_gridmap_core

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
  src/OccupancyGridMapMsgs.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)