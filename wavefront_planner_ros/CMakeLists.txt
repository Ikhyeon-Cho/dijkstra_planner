cmake_minimum_required(VERSION 3.0.2)
project(wavefront_planner)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  wavefront_planner_core
  occupancy_gridmap_msgs
  wavefront_planner_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    # wavefront_planner
  CATKIN_DEPENDS 
    roscpp
    wavefront_planner_core
    occupancy_gridmap_msgs
    wavefront_planner_msgs
)

###########
## Build ##
###########
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wavefront_planner_node src/wavefront_planner_node.cpp
  src/WavefrontPlanner.cpp
)

target_link_libraries(wavefront_planner_node
  ${catkin_LIBRARIES}
)

# add_executable(localgoal_generator_node src/localgoal_generator_node.cpp
#   src/local_goal_generator/LocalGoalGenerator.cpp
#   src/wavefront_planner/WavefrontPlannerRosConverter.cpp
# )
# target_link_libraries(localgoal_generator_node
#   ${catkin_LIBRARIES}
# )