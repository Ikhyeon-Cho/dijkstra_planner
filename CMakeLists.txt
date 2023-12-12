cmake_minimum_required(VERSION 3.0.2)
project(wavefront_planner)
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ros_node_utils
  ros_transform_utils
  elevation_map_server
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # wavefront_planner
  CATKIN_DEPENDS 
    roscpp
    ros_node_utils
    ros_transform_utils
    elevation_map_server
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

add_executable(waypoint_manager_node src/waypoint_manager_node.cpp
  # src/waypoint_manager/WaypointManager.cpp
  src/wavefront_planner/WavefrontPlannerRosConverter.cpp
)
target_link_libraries(waypoint_manager_node
  ${catkin_LIBRARIES}
)