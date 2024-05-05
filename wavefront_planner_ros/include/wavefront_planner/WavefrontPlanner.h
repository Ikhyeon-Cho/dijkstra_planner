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

#include <wavefront_core/wavefront_core.h>
#include <wavefront_msgs/WavefrontMsgs.h>

class WavefrontPlanner
{
public:
  WavefrontPlanner();

  void initializeMap(const nav_msgs::OccupancyGridConstPtr& msg);

  void wavePropagation(const geometry_msgs::PoseStampedConstPtr& msg);

  void findPath(const ros::TimerEvent& event);

  void debug(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_priv_{ "~" };
  ros_utils::TransformHandler tf_;

  // Topics
  std::string occupancy_map_topic_{ nh_priv_.param<std::string>("occupancyMapTopic", "/map") };
  std::string goal_topic_{ nh_priv_.param<std::string>("goalTopic", "/move_base_simple/goal") };

  // Frame Ids
  std::string baselink_frame{ nh_priv_.param<std::string>("baselinkFrame", "base_link") };
  std::string map_frame{ nh_priv_.param<std::string>("mapFrame", "map") };

  // Planner Options
  double inflation_radius_{ nh_priv_.param<double>("inflationRadius", 1.0) };
  bool enable_search_unknown_{ nh_priv_.param<bool>("propagateUnknownCell", false) };
  bool debug_mode_{ nh_priv_.param<bool>("debugMode", false) };
  // Duration
  int path_update_rate_{ nh_priv_.param<int>("pathPublishRate", 20) };

  // ROS
  ros::Subscriber sub_map_{ nh_priv_.subscribe(occupancy_map_topic_, 10, &WavefrontPlanner::initializeMap, this) };
  ros::Subscriber sub_goal_{ nh_priv_.subscribe(goal_topic_, 10, &WavefrontPlanner::wavePropagation, this) };
  ros::Publisher pub_path_{ nh_priv_.advertise<nav_msgs::Path>("path", 10) };

  ros::Timer path_finding_timer_{ nh_priv_.createTimer(path_update_rate_, &WavefrontPlanner::findPath, this, false,
                                                          false) };
  ros::Timer debug_timer_;
  ros::Publisher pub_costmap_;
  ros::Publisher pub_occupancy_map_;

private:
  WavePropagator wave_propagator_;
  OccupancyMap occupancy_map_;
  geometry_msgs::PoseStamped goal_;

  bool map_is_initialized_;
};

#endif  // WAVEFRONT_PLANNER_H