/*
 * WavefrontPlannerRosConverter.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef WAVEFRONT_PLANNER_ROS_CONVERTER_H
#define WAVEFRONT_PLANNER_ROS_CONVERTER_H

#include "wavefront_planner/WavefrontPlanner.h"
#include <nav_msgs/Path.h>

class WavefrontPlannerRosConverter
{
public:
  static void toPathMsg(const std::vector<Eigen::Vector2d>& path, nav_msgs::Path& msg);

  static void fromPathMsg(const nav_msgs::Path& msg, std::vector<Eigen::Vector2d>& path);

  static void fromPoseMsg(const geometry_msgs::Pose& msg, Eigen::Vector2d& position);

  static void toPoseMsg(const Eigen::Vector2d& position, geometry_msgs::PoseStamped& msg);
};

#endif