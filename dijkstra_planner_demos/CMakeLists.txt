cmake_minimum_required(VERSION 3.0.2)
project(dijkstra_planner_demos)
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
    # dijkstra_planner_demos
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

# Search Demo node
add_executable(dijkstra_planner_demos 
  src/dijkstra_planner_demos_node.cpp
  src/DijkstraPlannerDemo.cpp # Wavefront search demo
  src/CostmapDemo.cpp # Potential field demo
  src/WavePropagatorDemo.cpp
)
target_link_libraries(dijkstra_planner_demos
  ${catkin_LIBRARIES}
)