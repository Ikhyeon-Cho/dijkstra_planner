/*
 * CostmapDemo.cpp
 *
 *  Created on: May 5, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "dijkstra_planner_demos/CostmapDemo.h"

CostmapDemo::CostmapDemo()
{
  // Read images to generate the occupancy map
  auto img = cv::imread(map_img_dir_, cv::IMREAD_GRAYSCALE);
  OccupancyMapHelper::initializeFromImage(img, grid_resolution_, occupancy_map_);

  // Visualize the occupancy map
  nav_msgs::OccupancyGrid occu_map_msg;
  OccupancyMapConverter::toROSMsg(occupancy_map_, occu_map_msg);
  pub_occupancy_map_.publish(occu_map_msg);

  // Publish the robot and goal position
  geometry_msgs::PoseStamped robot_pose;
  robot_pose.header.frame_id = map_frame;
  robot_pose.pose.position.x = robot_x_;
  robot_pose.pose.position.y = robot_y_;
  pub_robot_.publish(robot_pose);

  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = map_frame;
  goal_pose.pose.position.x = goal_x_;
  goal_pose.pose.position.y = goal_y_;
  pub_goal_.publish(goal_pose);

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

void CostmapDemo::search()
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
    ROS_WARN("%-65s %s\n", "Invalid search. Please check the map.", "[ CostmapDemo]");
  }

  // Wave propagation
  if (!wave_propagator_.wavePropagation(goal_index, robot_index, pub_costmap_, 20.0))
  {
    // ros::shutdown();
  }
  wave_propagator_.getCostMap()["cost_wave_expansion"] /= 3.0;
  path_generation_timer_.start();
}

void CostmapDemo::publish(const ros::TimerEvent& event)
{
  if (pub_costmap_.getNumSubscribers() > 0)
  {
    grid_map_msgs::GridMap costmap_msg;
    auto& costmap = wave_propagator_.getCostMap();
    grid_map::GridMapRosConverter::toMessage(costmap, costmap_msg);
    costmap_msg.info.header.frame_id = map_frame;
    pub_costmap_.publish(costmap_msg);
  }

  if (pub_costmap_.getNumSubscribers() > 0)
  {
    nav_msgs::OccupancyGrid occu_map_msg;
    OccupancyMapConverter::toROSMsg(occupancy_map_, occu_map_msg);
    pub_occupancy_map_.publish(occu_map_msg);
  }
}

void CostmapDemo::findPath(const ros::TimerEvent& event)
{
  // Set the goal and robot position from the launch file param
  auto goal_position = grid_map::Position(goal_x_, goal_y_);
  auto robot_position = grid_map::Position(robot_x_, robot_y_);

  // Generate current path on the basis of the costmap
  auto [has_path, path] = wave_propagator_.findPath(robot_position, goal_position);
  if (has_path)  // 1. Path to the goal is already searched from the current robot position
  {
    nav_msgs::Path msg_path;
    DijkstraPlannerMsgs::toPathMsg(path, msg_path);
    pub_path_.publish(msg_path);
    return;
  }
  else  // 2. If path generation currently fails, then re-activate wave propagation
  {
    ROS_WARN("%-65s %s", "Failed to generate path from the costmap. Replanning...", "[ CostmapDemo]");
    if (!wave_propagator_.search(goal_position, robot_position))
    {
      ROS_ERROR("%-65s %s\n", "Could not complete the search. No path available.", "[CostmapDemo]");
      return;
    }

    ROS_INFO("%-65s %s\n", "Replanning done. Found a new path", "[ CostmapDemo]");
    return;
  }
}