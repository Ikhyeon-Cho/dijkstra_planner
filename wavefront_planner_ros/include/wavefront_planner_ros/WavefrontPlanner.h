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

// Occupancy GridMap
#include <occupancy_gridmap_core/OccupancyGridMap.h>
#include <occupancy_gridmap_core/OccupancyGridMapHelper.h>
#include <occupancy_gridmap_msgs/OccupancyGridMapMsgs.h>

// CostMap
#include <costmap_core/CostMap.h>
#include <costmap_core/CostMapHelper.h>

#include "wavefront_planner_core/WavePropagator.h"
#include "wavefront_planner_msgs/WavefrontPlannerMsgs.h"

class WavefrontPlanner
{
public:
  WavefrontPlanner();

  void defineSearchspace(const nav_msgs::OccupancyGridConstPtr& msg);

  void doWavePropagation(const geometry_msgs::PoseStampedConstPtr& msg);

  void generatePath(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_{ "~" };
  ros_utils::TransformHandler tf_handler_;

  // Topics
  std::string occupancy_gridmap_topic_{ nh_.param<std::string>("occupancyGridMapTopic", "/map_inflated") };
  std::string goal_topic_{ nh_.param<std::string>("goalTopic", "/move_base_simple/goal") };
  std::string path_topic_{ nh_.param<std::string>("pathTopic", "path") };

  // Frame Ids
  std::string baselink_frame_{ nh_.param<std::string>("baselinkFrame", "base_link") };
  std::string map_frame_{ nh_.param<std::string>("mapFrame", "map") };

  // Search options
  bool search_unknown_space_{ nh_.param<bool>("searchUnknownSpace", false) };

  // Duration
  int path_update_rate_{ nh_.param<int>("pathUpdateRate", 20) };

  // ROS
  ros::Subscriber sub_map_{ nh_.subscribe(occupancy_gridmap_topic_, 10, &WavefrontPlanner::defineSearchspace, this) };
  ros::Subscriber sub_goal_{ nh_.subscribe(goal_topic_, 10, &WavefrontPlanner::doWavePropagation, this) };
  ros::Publisher pub_path_{ nh_.advertise<nav_msgs::Path>(path_topic_, 10) };
  ros::Publisher pub_costmap_{ nh_.advertise<grid_map_msgs::GridMap>("costmap", 1) };

  ros::Timer path_generation_timer_{ nh_.createTimer(path_update_rate_, &WavefrontPlanner::generatePath, this, false,
                                                     false) };

private:
  WavePropagator wave_propagator_;
  geometry_msgs::PoseStamped goal_;

  bool map_is_initialized_{ false };
};

#endif  // WAVEFRONT_PLANNER_H