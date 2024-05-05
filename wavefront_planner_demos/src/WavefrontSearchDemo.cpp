/*
 * WavefrontSearchDemo.cpp
 *
 *  Created on: May 5, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "wavefront_planner_demos/WavefrontSearchDemo.h"

WavefrontSearchDemo::WavefrontSearchDemo()
{
  // Read images to generate the occupancy map
  auto img = cv::imread(map_img_dir_, cv::IMREAD_GRAYSCALE);
  OccupancyMapHelper::initializeFromImage(img, grid_resolution_, occupancy_map_);

  // Visualize the occupancy map
  nav_msgs::OccupancyGrid occu_map_msg;
  OccupancyMapConverter::toROSMsg(occupancy_map_, occu_map_msg);
  pub_occupancy_map_.publish(occu_map_msg);

  ros::Duration duration(2.0);
  duration.sleep();

  // Initialize the wave propagator
  wave_propagator_.setSafeDistance(safe_distance_);
  wave_propagator_.initialize(occupancy_map_, enable_search_unknown_);

  // Add cost_elevation layer for visualization
  wave_propagator_.getCostMap().add("cost_elevation", 0.0);

  search();

  publish_timer_.start();
}

void WavefrontSearchDemo::publish(const ros::TimerEvent& event)
{
  // Visualize cost map
  if (pub_costmap_.getNumSubscribers() > 0)
  {
    grid_map_msgs::GridMap costmap_msg;
    grid_map::GridMapRosConverter::toMessage(wave_propagator_.getCostMap(), costmap_msg);
    costmap_msg.info.header.frame_id = map_frame;
    pub_costmap_.publish(costmap_msg);
  }

  // Visualize occupancy map
  if (pub_costmap_.getNumSubscribers() > 0)
  {
    nav_msgs::OccupancyGrid occu_map_msg;
    OccupancyMapConverter::toROSMsg(occupancy_map_, occu_map_msg);
    pub_occupancy_map_.publish(occu_map_msg);
  }
}

void WavefrontSearchDemo::search()
{
  // Set the goal and robot position from the launch file param
  auto goal_position = grid_map::Position(goal_x_, goal_y_);
  auto robot_position = grid_map::Position(robot_x_, robot_y_);

  // Get the grid index from the position
  const auto costmap = wave_propagator_.getCostMap();
  auto goal_index = costmap.getGridIndexFrom(goal_position);
  auto robot_index = costmap.getGridIndexFrom(robot_position);

  // Check the validity of the robot and goal position
  if (!wave_propagator_.isValidsearch(goal_position, robot_position))
  {
    ROS_WARN("%-65s %s\n", "Invalid search. Please check the map.", "[ WavefrontSearchDemo]");
  }

  // Wave propagation
  if (!wave_propagator_.wavePropagation(goal_index, robot_index, pub_costmap_, 3.0))
  {
    ros::shutdown();
  }
}