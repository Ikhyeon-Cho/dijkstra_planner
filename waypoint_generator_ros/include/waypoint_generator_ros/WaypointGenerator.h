/*
 * WaypointGenerator.h
 *
 *  Created on: Dec 12, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef WAYPOINT_GENERATOR_H
#define WAYPOINT_GENERATOR_H

#include <ros/ros.h>
#include "dijkstra_planner_msgs/Msgs.h"
#include "ros_utils/TransformHandler.h"
#include "ros_utils/transform.h"

class WaypointGenerator
{
public:
  WaypointGenerator();

  void findWaypoint(const nav_msgs::PathConstPtr& msg);

private:
  ros::NodeHandle nh_{ "~" };

  // Topics
  std::string path_topic_{ nh_.param<std::string>("/dijkstra_planner/pathTopic", "path") };
  std::string waypoint_topic_{ nh_.param<std::string>("waypointTopic", "waypoint_goal") };

  // Frame Ids
  std::string map_frame_{ nh_.param<std::string>("/dijkstra_planner/mapFrame", "map") };
  std::string baselink_frame_{ nh_.param<std::string>("/dijkstra_planner/baselinkFrame", "base_link") };

  // Parameters
  double distance_from_robot_{ nh_.param<double>("distanceFromRobot", 5.0) };

  // ROS
  ros::Subscriber sub_path_{ nh_.subscribe("/dijkstra_planner/" + path_topic_, 1, &WaypointGenerator::findWaypoint,
                                           this) };
  ros::Publisher pub_waypoint_{ nh_.advertise<geometry_msgs::PoseStamped>(waypoint_topic_, 10) };

  // ROS Utils
  ros_utils::TransformHandler transform_handler_;
};

#endif  // WAYPOINT_GENERATOR_H