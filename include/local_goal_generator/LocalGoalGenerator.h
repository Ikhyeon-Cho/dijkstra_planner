/*
 * LocalGoalGenerator.h
 *
 *  Created on: Dec 12, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_LOCAL_GOAL_GENERATOR_H
#define ROS_LOCAL_GOAL_GENERATOR_H

#include <ros/ros.h>
#include <ros_node_utils/Core.h>
#include <ros_transform_utils/TransformHandler.h>

#include <nav_msgs/Path.h>
#include "wavefront_planner/WavefrontPlannerRosConverter.h"

namespace ros
{
class LocalGoalGenerator
{
public:
  LocalGoalGenerator();

  void findLocalGoal(const nav_msgs::PathConstPtr& msg);

public:
  // Subscribed Topics
  roscpp::Parameter<std::string> globalPath_topic{ "~/Subscribed_Topics/global_path" , "path"};

  // Published Topics
  roscpp::Parameter<std::string> localGoal_topic{ "~/Published_Topics/local_goal", "goal" };

  // Parameters
  // -- Frame Ids
  roscpp::Parameter<std::string> base_frameId{ "~/Parameters/base_frameId", "base_link" };
  roscpp::Parameter<std::string> map_frameId{ "~/Parameters/map_frameId", "map" };
  // -- Local Goal
  roscpp::Parameter<double> distance_to_localGoal{ "~/Parameters/distance_to_localGoal", 5.0 };

private:
  TransformHandler transform_handler_;

  roscpp::Subscriber<nav_msgs::Path> global_path_subscriber{ globalPath_topic.param(),
                                                             &LocalGoalGenerator::findLocalGoal, this };

  roscpp::Publisher<geometry_msgs::PoseStamped> local_goal_publisher{ localGoal_topic.param() };
};

}  // namespace ros

#endif  // ROS_LOCAL_GOAL_GENERATOR_H