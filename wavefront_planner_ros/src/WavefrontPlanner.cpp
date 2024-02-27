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
  if (use_unknown_as_free_)
    wavefront_planner_.searchUnknownArea(true);
}

void WavefrontPlanner::setGridSearchSpace(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if (already_received_map_)
    return;

  // Convert nav_msgs::OccupancyGrid to grid_map::OccupancyGridMap
  OccupancyGridMap occupancy_map;
  OccupancyGridMapMsgs::fromOccupancyGridMsg(*msg, occupancy_map);

  // Set grid search space
  wavefront_planner_.setGridSearchSpace(occupancy_map);
  already_received_map_ = true;
  std::cout << "test \n";
}

void WavefrontPlanner::doWavePropagation(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // Check whether the msg is in the map frame - if not, transform it
  if (msg->header.frame_id == map_frame_)
  {
    goalpose_in_map_ = *msg;
  }
  else
  {
    ROS_WARN(" [WavefrontPlanner::doWavePropagation] Received Goal is not in the map frame. Transforming...");
    auto [has_transform, transform_to_map] = tf_handler_.getTransform(msg->header.frame_id, map_frame_);
    if (!has_transform)
    {
      ROS_WARN(" [WavefrontPlanner::doWavePropagation] Failed to get the transform from %s to %s",
               msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }

    if (!ros_utils::tf::doTransform(*msg, goalpose_in_map_, transform_to_map))
    {
      ROS_WARN(" [WavefrontPlanner::doWavePropagation] Failed to transform goal pose from %s to %s",
               msg->header.frame_id.c_str(), map_frame_.c_str());
      return;
    }
  }

  // Set robot position in map
  auto [has_transform_b2m, base_to_map] = tf_handler_.getTransform(baselink_frame_, map_frame_);
  if (!has_transform_b2m)
  {
    ROS_WARN(" [WavefrontPlanner::doWavePropagation] Failed to get robot pose in the map frame. Skip path planning");
    return;
  }

  // Convert geometry_msgs::TransformStamped to Eigen::Vector2d
  Eigen::Vector2d goal_position{ goalpose_in_map_.pose.position.x, goalpose_in_map_.pose.position.y };
  Eigen::Vector2d robot_position{ base_to_map.transform.translation.x, base_to_map.transform.translation.y };

  if (!wavefront_planner_.doWavePropagationAt(goal_position, robot_position))
  {
    ROS_WARN(" [WavefrontPlanner::doWavePropagation] Could not find feasible Path from current pose to goal.");
    return;
  }

  ROS_INFO("Path found. Start Generating path");
  path_generation_timer_.start();

  // for debug purpose
  grid_map_msgs::GridMap costmap_msg;
  grid_map::GridMapRosConverter::toMessage(*wavefront_planner_.getCostMap(), costmap_msg);
  costmap_msg.info.header.frame_id = map_frame_;
  pub_costmap_.publish(costmap_msg);
}

void WavefrontPlanner::generatePath(const ros::TimerEvent& event)
{
  // Set received goal position
  Eigen::Vector2d goal_position;
  WavefrontPlannerMsgs::fromPoseMsg(goalpose_in_map_, goal_position);

  // Update robot position
  auto [has_transform, base_to_map] = tf_handler_.getTransform(baselink_frame_, map_frame_);
  if (!has_transform)
  {
    ROS_WARN(" [WavefrontPlanner::generatePath] Failed to get robot pose in the map frame. Skip generating path");
    return;
  }
  Eigen::Vector2d robot_position{ base_to_map.transform.translation.x, base_to_map.transform.translation.y };

  // Generate current path
  std::vector<Eigen::Vector2d> path;
  bool can_update_path = wavefront_planner_.doPathGeneration(robot_position, goal_position, path);
  if (can_update_path)  // 1. Path to Goal is available from current robot position
  {
    nav_msgs::Path msg_path;
    WavefrontPlannerMsgs::toPathMsg(path, msg_path);
    pub_path_.publish(msg_path);
  }
  else  // 2. If path generation currently fails, then again do wave propagation
  {
    ROS_WARN(" [WavefrontPlanner::generatePath] Failed to generate path. Replanning...");
    if (!wavefront_planner_.doWavePropagationAt(goal_position, robot_position))
    {
      ROS_WARN(" [WavefrontPlanner::generatePath] Could not find feasible Path from current pose to goal.");
      return;
    }

    ROS_INFO("Replanning done. found path");
  }
}