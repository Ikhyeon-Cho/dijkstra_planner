/*
 * WavefrontPlanner.cpp
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "wavefront_planner_ros/WavefrontPlanner.h"

WavefrontPlanner::WavefrontPlanner()
{
  // Print Node Info
  std::cout << std::endl;
  ROS_INFO("%-65s %s", ("Subscribing to the map topic: " + occupancy_gridmap_topic_).c_str(), "[ WavefrontPlanner]");
  ROS_INFO("%-65s %s", ("Subscribing to the goal topic: " + goal_topic_).c_str(), "[ WavefrontPlanner]");
  std::cout << std::endl;
}

void WavefrontPlanner::defineSearchspace(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if (map_is_initialized_)
    return;

  // Convert nav_msgs::OccupancyGrid to OccupancyGridMap
  OccupancyGridMap occupancy_map;
  OccupancyGridMapConverter::fromOccupancyGridMsg(*msg, occupancy_map);

  // Convert OccupancyGridMap to CostMap
  CostMap costmap;
  CostMapHelper::fromOccupancyGridMap(occupancy_map, costmap, search_unknown_space_);

  // Set grid search space
  wave_propagator_.initialize(costmap);
  map_is_initialized_ = true;
}

void WavefrontPlanner::doWavePropagation(const geometry_msgs::PoseStampedConstPtr& msg)
{
  ROS_INFO("%-65s %s", "Received a Goal.", "[ WavefrontPlanner]");

  // Check whether the map is received
  if (!map_is_initialized_)
  {
    ROS_WARN("%-65s %s\n", "Map is not received yet --> Check the map topic", "[ WavefrontPlanner]");
    return;
  }

  // Check whether the goal msg is in the map frame - if not, transform it
  if (msg->header.frame_id == map_frame_)
  {
    goal_ = *msg;
  }
  else
  {
    ROS_WARN("%-65s %s", "Goal is not in the map frame. Transforming...", "[ WavefrontPlanner]");

    auto [has_transform, transform_to_map] = tf_handler_.getTransform(msg->header.frame_id, map_frame_);
    if (!has_transform)
    {
      ROS_WARN("%-65s %s\n", "Transform Failed. Please check TF Tree connection!", "[ WavefrontPlanner]");
      return;
    }

    ros_utils::tf::doTransform(*msg, goal_, transform_to_map);
    ROS_WARN("%-65s %s", "Transform Succeeded.", "[ WavefrontPlanner]");
  }

  // Check whether the robot is in the map frame
  auto [has_transform_b2m, base_to_map] = tf_handler_.getTransform(baselink_frame_, map_frame_);
  if (!has_transform_b2m)
  {
    ROS_WARN("%-65s %s\n", "Could not get the Robot Pose. Please check TF Tree connection!", "[ WavefrontPlanner]");
    return;
  }

  // Convert geometry_msgs::TransformStamped to grid_map::Position
  grid_map::Position goal_position{ goal_.pose.position.x, goal_.pose.position.y };
  grid_map::Position robot_position{ base_to_map.transform.translation.x, base_to_map.transform.translation.y };

  if (!wave_propagator_.doWavePropagationAt(goal_position, robot_position))
  {
    ROS_ERROR("%-65s %s\n", "No Feasible Path found. The planner will not publish the path.", "[ WavefrontPlanner]");
    path_generation_timer_.stop();
    return;
  }

  ROS_INFO("%-65s %s\n", ("Path found. Publishing the path in " + std::to_string(path_update_rate_) + " Hz...").c_str(),
           "[ WavefrontPlanner]");
  path_generation_timer_.start();

  // For Debug purpose: Visualize the costmap
  if (pub_costmap_.getNumSubscribers() > 0)
  {
    grid_map_msgs::GridMap costmap_msg;
    grid_map::GridMapRosConverter::toMessage(wave_propagator_.getCostMap(), costmap_msg);
    costmap_msg.info.header.frame_id = map_frame_;
    pub_costmap_.publish(costmap_msg);
  }
}

void WavefrontPlanner::generatePath(const ros::TimerEvent& event)
{
  // Set the received goal position
  grid_map::Position goal_position;
  WavefrontPlannerMsgs::fromPoseMsg(goal_, goal_position);

  // Check the availability of the robot position
  // We assume that the robot navigates after planning -> Position is keep changing
  // Update the robot position
  grid_map::Position robot_position;
  auto [has_transform, base_to_map] = tf_handler_.getTransform(baselink_frame_, map_frame_);
  robot_position << base_to_map.transform.translation.x, base_to_map.transform.translation.y;
  if (!has_transform)
  {
    ROS_ERROR("%-65s %s\n", "Could not get robot pose. Please check TF Tree connection!", "[ WavefrontPlanner]");
    return;
  }

  // Generate current path on the basis of the wavefront costmap
  auto [has_path, path] = wave_propagator_.doPathGeneration(robot_position, goal_position);
  if (has_path)  // 1. Path to the goal is already searched from the current robot position
  {
    nav_msgs::Path msg_path;
    WavefrontPlannerMsgs::toPathMsg(path, msg_path);
    pub_path_.publish(msg_path);
  }
  else  // 2. If path generation currently fails, then re-activate wave propagation
  {
    ROS_WARN("%-65s %s", "Failed to generate path from the costmap. Replanning...", "[ WavefrontPlanner]");
    if (!wave_propagator_.doWavePropagationAt(goal_position, robot_position))
    {
      ROS_ERROR("%-65s %s\n", "Could not complete the wavefront search. The planner will not publish the path.",
                "[ WavefrontPlanner]");
      return;
    }

    ROS_INFO("%-65s %s\n", "Replanning done. Found a new path", "[ WavefrontPlanner]");

    // For Debug purpose: Visualize the costmap
    if (pub_costmap_.getNumSubscribers() > 0)
    {
      grid_map_msgs::GridMap costmap_msg;
      grid_map::GridMapRosConverter::toMessage(wave_propagator_.getCostMap(), costmap_msg);
      costmap_msg.info.header.frame_id = map_frame_;
      pub_costmap_.publish(costmap_msg);
    }
  }
}