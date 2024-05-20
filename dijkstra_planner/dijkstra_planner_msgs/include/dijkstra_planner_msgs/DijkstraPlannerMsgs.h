/*
 * DijkstraPlannerMsgs.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef DIJKSTRA_PLANNER_MSGS_H
#define DIJKSTRA_PLANNER_MSGS_H

#include <vector>
#include <Eigen/Dense>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <grid_map_msgs/GridMap.h>

class DijkstraPlannerMsgs
{
public:
  static void toPathMsg(const std::vector<Eigen::Vector2d>& path, nav_msgs::Path& msg);

  static void fromPathMsg(const nav_msgs::Path& msg, std::vector<Eigen::Vector2d>& path);

  static void fromPoseMsg(const geometry_msgs::Pose& msg, Eigen::Vector2d& position);

  static void fromPoseMsg(const geometry_msgs::PoseStamped& msg, Eigen::Vector2d& position);

  static void toPoseMsg(const Eigen::Vector2d& position, geometry_msgs::PoseStamped& msg);
};

#endif  // DIJKSTRA_PLANNER_MSGS_H