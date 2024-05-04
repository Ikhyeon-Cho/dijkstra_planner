/*
 * WavefrontPlanner.cpp
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "wavefront_planner/WavefrontPlanner.h"

WavefrontPlanner::WavefrontPlanner() : map_is_initialized_{ false }
{
  // Print Node Info
  std::cout << std::endl;
  ROS_INFO("%-65s %s", ("Subscribing to the map topic: " + occupancy_map_topic_).c_str(), "[ WavefrontPlanner]");
  ROS_INFO("%-65s %s", ("Subscribing to the goal topic: " + goal_topic_).c_str(), "[ WavefrontPlanner]");
  std::cout << std::endl;
}

void WavefrontPlanner::initializeMap(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if (map_is_initialized_)
    return;

  OccupancyMapConverter::fromROSMsg(*msg, occupancy_map_);

  // Set grid search space
  wave_propagator_.setSafeDistance(inflation_radius_);
  wave_propagator_.initialize(occupancy_map_, enable_search_unknown_);
  
  map_is_initialized_ = true;

  if (debug_mode_)
  {
    ROS_INFO("%-65s %s", "Map is initialized. Starting Debug Mode...", "[ WavefrontPlanner]");
    debug_timer_.start();
  }
  else
  {
    ROS_INFO("%-65s %s", "Map is initialized. Waiting for the goal...", "[ WavefrontPlanner]");
  }
}

void WavefrontPlanner::wavePropagation(const geometry_msgs::PoseStampedConstPtr& msg)
{
  ROS_INFO("%-65s %s", "Received a Goal.", "[ WavefrontPlanner]");

  // Check whether the map is received
  if (!map_is_initialized_)
  {
    ROS_WARN("%-65s %s\n", "Map is not received yet --> Check the map topic", "[ WavefrontPlanner]");
    return;
  }

  // Check whether the goal msg is in the map frame - if not, transform it
  if (msg->header.frame_id == map_frame)
  {
    goal_ = *msg;
  }
  else
  {
    auto [has_transform, transform_to_map] = tf_.getTransform(msg->header.frame_id, map_frame);
    if (!has_transform)
    {
      ROS_WARN("%-65s %s\n", "Transform Failed. Goal was not given in map frame!", "[ WavefrontPlanner]");
      return;
    }

    ros_utils::tf::doTransform(*msg, goal_, transform_to_map);
  }

  // Check current robot pose
  auto [has_transform_b2m, base_to_map] = tf_.getTransform(baselink_frame, map_frame);
  if (!has_transform_b2m)
  {
    ROS_WARN("%-65s %s\n", "Could not get the Robot Pose. Please check TF Tree connection!", "[ WavefrontPlanner]");
    return;
  }

  // Convert geometry_msgs::TransformStamped to grid_map::Position
  grid_map::Position goal_position{ goal_.pose.position.x, goal_.pose.position.y };
  grid_map::Position robot_position{ base_to_map.transform.translation.x, base_to_map.transform.translation.y };

  if (!wave_propagator_.search(goal_position, robot_position))
  {
    ROS_ERROR("%-65s %s\n", "No Feasible Path found.", "[ WavefrontPlanner]");
    path_generation_timer_.stop();
    return;
  }

  ROS_INFO("%-65s %s\n", ("Path found. Publishing the path in " + std::to_string(path_update_rate_) + " Hz...").c_str(),
           "[ WavefrontPlanner]");
  path_generation_timer_.start();
}

void WavefrontPlanner::findPath(const ros::TimerEvent& event)
{
  // Set the received goal position
  grid_map::Position goal_position;
  WavefrontPlannerMsgs::fromPoseMsg(goal_, goal_position);

  // Check the validity of the robot position
  // We assume that the robot navigates after planning -> Position is keep changing
  auto [has_transform, base_to_map] = tf_.getTransform(baselink_frame, map_frame);
  if (!has_transform)
  {
    ROS_ERROR("%-65s %s\n", "Could not get robot pose. Please check TF Tree connection!", "[ WavefrontPlanner]");
    return;
  }
  grid_map::Position robot_position;
  robot_position << base_to_map.transform.translation.x, base_to_map.transform.translation.y;

  // Generate current path on the basis of the wavefront costmap
  auto [has_path, path] = wave_propagator_.findPath(robot_position, goal_position);
  if (has_path)  // 1. Path to the goal is already searched from the current robot position
  {
    nav_msgs::Path msg_path;
    WavefrontPlannerMsgs::toPathMsg(path, msg_path);
    pub_path_.publish(msg_path);
    return;
  }
  else  // 2. If path generation currently fails, then re-activate wave propagation
  {
    ROS_WARN("%-65s %s", "Failed to generate path from the costmap. Replanning...", "[ WavefrontPlanner]");
    if (!wave_propagator_.search(goal_position, robot_position))
    {
      ROS_ERROR("%-65s %s\n", "Could not complete the wavefront search. No path available.", "[ WavefrontPlanner]");
      return;
    }

    ROS_INFO("%-65s %s\n", "Replanning done. Found a new path", "[ WavefrontPlanner]");
    return;
  }
}

void WavefrontPlanner::debug(const ros::TimerEvent& event)
{
  // For Debug purpose: Visualize the costmap
  if (pub_costmap_.getNumSubscribers() > 0)
  {
    grid_map_msgs::GridMap costmap_msg;
    grid_map::GridMapRosConverter::toMessage(wave_propagator_.getCostMap(), costmap_msg);
    costmap_msg.info.header.frame_id = map_frame;
    pub_costmap_.publish(costmap_msg);
  }

  // For Debug purpose: Visualize the occupancy map
  if (pub_occupancy_map_.getNumSubscribers() > 0)
  {
    nav_msgs::OccupancyGrid occu_map_msg;
    OccupancyMapConverter::toROSMsg(occupancy_map_, occu_map_msg);
    pub_occupancy_map_.publish(occu_map_msg);
  }
}