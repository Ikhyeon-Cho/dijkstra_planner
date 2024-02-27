cmake_minimum_required(VERSION 3.0.2)
project(occupancy_gridmap_server)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  occupancy_gridmap_core
  occupancy_gridmap_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # wavefront_planner
  CATKIN_DEPENDS 
    roscpp
    occupancy_gridmap_core
    occupancy_gridmap_msgs
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(occupancy_gridmap_server
  src/gridmap_server_node.cpp
  src/GridMapServer.cpp
)

target_link_libraries(occupancy_gridmap_server
  ${catkin_LIBRARIES}
)