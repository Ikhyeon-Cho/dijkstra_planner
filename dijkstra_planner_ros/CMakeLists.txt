cmake_minimum_required(VERSION 3.0.2)
project(dijkstra_planner)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  dijkstra_planner_core
  dijkstra_planner_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # dijkstra_planner
  CATKIN_DEPENDS 
    roscpp
    tf2_ros
    dijkstra_planner_core
    dijkstra_planner_msgs
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dijkstra_planner_node 
  src/dijkstra_planner_node.cpp
  src/DijkstraPlanner.cpp
)

target_link_libraries(dijkstra_planner_node
  ${catkin_LIBRARIES}
)