/*
 * WavefrontPlannerRosConverter.cpp
 *
 *  Created on: Nov 28, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "wavefront_planner/WavefrontPlannerRosConverter.h"

void WavefrontPlannerRosConverter::toPathMsg(const std::vector<Eigen::Vector2d>& path, nav_msgs::Path& msg)
{
  msg.poses.clear();
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();

  for (const auto& position : path)
  {
    geometry_msgs::PoseStamped pose;
    pose.header = msg.header;
    pose.pose.position.x = position.x();
    pose.pose.position.y = position.y();
    pose.pose.position.z = 0;

    msg.poses.push_back(pose);
  }
}

void WavefrontPlannerRosConverter::fromPathMsg(const nav_msgs::Path& msg, std::vector<Eigen::Vector2d>& path)
{
  for (const auto& poseStamped : msg.poses)
  {
    Eigen::Vector2d position;
    position.x() = poseStamped.pose.position.x;
    position.y() = poseStamped.pose.position.y;
    path.push_back(position);
  }
}

void WavefrontPlannerRosConverter::fromPoseMsg(const geometry_msgs::Pose& msg, Eigen::Vector2d& position)
{
  position.x() = msg.position.x;
  position.y() = msg.position.y;
}

void WavefrontPlannerRosConverter::toPoseMsg(const Eigen::Vector2d& position, geometry_msgs::PoseStamped& msg)
{
  msg.header.frame_id = "map";
  msg.header.stamp = ros::Time::now();

  msg.pose.position.x = position.x();
  msg.pose.position.y = position.y();
  msg.pose.position.z = 0;
}