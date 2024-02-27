/*
 * WavefrontPlanner.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef WAVEFRONT_PLANNER_H
#define WAVEFRONT_PLANNER_H

#include <ros/ros.h>

// Utility
#include "ros_utils/TransformHandler.h"
#include "ros_utils/transform.h"

#include <occupancy_gridmap_core/OccupancyGridMap.h>
#include <occupancy_gridmap_core/OccupancyGridMapHelper.h>
#include <occupancy_gridmap_msgs/OccupancyGridMapMsgs.h>

#include "wavefront_planner_core/WavePropagator.h"
#include "wavefront_planner_msgs/WavefrontPlannerMsgs.h"

class WavefrontPlanner
{
public:
  WavefrontPlanner();

  void setGridSearchSpace(const nav_msgs::OccupancyGridConstPtr& msg);

  void doWavePropagation(const geometry_msgs::PoseStampedConstPtr& msg);

  void generatePath(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_{ "~" };
  ros_utils::TransformHandler tf_handler_;

  // Topics
  std::string occupancy_gridmap_topic_{ nh_.param<std::string>("occupancyGridMapTopic", "/map") };
  std::string goal_topic_{ nh_.param<std::string>("goalTopic", "/move_base_simple/goal") };
  std::string path_topic_{ nh_.param<std::string>("pathTopic", "path") };

  // Frame Ids
  std::string baselink_frame_{ nh_.param<std::string>("baselinkFrame", "base_link") };
  std::string map_frame_{ nh_.param<std::string>("mapFrame", "map") };

  // Path planner parameters
  bool use_unknown_as_free_{ nh_.param<bool>("useUnknownAsFree", false) };

  // Duration
  double path_update_rate_{ nh_.param<double>("pathUpdateRate", 20.0) };

  // ROS
  ros::Subscriber sub_map_{ nh_.subscribe(occupancy_gridmap_topic_, 10, &WavefrontPlanner::setGridSearchSpace, this) };
  ros::Subscriber sub_goal_{ nh_.subscribe(goal_topic_, 10, &WavefrontPlanner::doWavePropagation, this) };
  ros::Publisher pub_path_{ nh_.advertise<nav_msgs::Path>(path_topic_, 10) };
  ros::Publisher pub_occupancy_gridmap_{ nh_.advertise<nav_msgs::OccupancyGrid>(occupancy_gridmap_topic_, 10) };
  ros::Publisher pub_costmap_{ nh_.advertise<grid_map_msgs::GridMap>("costmap", 1) };

  ros::Timer path_generation_timer_{ nh_.createTimer(path_update_rate_, &WavefrontPlanner::generatePath, this, false,
                                                     false) };

private:
  bool already_received_map_{ false };

  OccupancyGridMap occupancy_map_;
  OccupancyGridMap inflated_map_;

  WavePropagator wavefront_planner_;

  geometry_msgs::PoseStamped goalpose_in_map_;
};

#endif  // WAVEFRONT_PLANNER_H