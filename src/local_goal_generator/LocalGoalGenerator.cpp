/*
 * LocalGoalGenerator.cpp
 *
 *  Created on: Dec 12, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "local_goal_generator/LocalGoalGenerator.h"

namespace ros
{

LocalGoalGenerator::LocalGoalGenerator()
{
}

void LocalGoalGenerator::findLocalGoal(const nav_msgs::PathConstPtr& msg)
{
  nav_msgs::Path path_in_localFrame;

  for (const auto& waypoint_in_mapFrame : msg->poses)
  {
    geometry_msgs::PoseStamped waypoint_in_baseFrame;
    if (!transform_handler_.doTransform(waypoint_in_mapFrame, base_frameId.param(), waypoint_in_baseFrame))
      return;

    Eigen::Vector2d waypoint_position;
    WavefrontPlannerRosConverter::fromPoseMsg(waypoint_in_baseFrame.pose, waypoint_position);

    if (waypoint_position.norm() > distance_to_localGoal.param())
      break;

    path_in_localFrame.poses.push_back(waypoint_in_baseFrame);
  }
  path_in_localFrame.header = msg->header;
  path_in_localFrame.header.frame_id = base_frameId.param();

  local_goal_publisher.publish(path_in_localFrame.poses.back());
}

}  // namespace ros