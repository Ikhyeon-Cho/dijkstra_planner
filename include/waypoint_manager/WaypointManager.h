/*
 * WaypointManager.h
 *
 *  Created on: Dec 12, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef ROS_WAYPOINT_MANAGER_H
#define ROS_WAYPOINT_MANAGER_H

#include <ros/ros.h>
#include <ros_node_utils/Core.h>
#include <ros_transform_utils/TransformHandler.h>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include "waypoint_manager/PathSampler.h"
#include "wavefront_planner/WavefrontPlannerRosConverter.h"

namespace ros
{
class WaypointManager
{
public:
  WaypointManager();

  void doPathSampling(const nav_msgs::PathConstPtr& msg);

  void doArrivalMonitoring(const ros::TimerEvent& event);

  // void

public:
  // Subscribed Topics
  roscpp::Parameter<std::string> goal_topic{ "~/Subscriber_Topics/goal", "/move_base_simple/goal" };
  roscpp::Parameter<std::string> path_topic{ "wavefront_planner_node/path" };

  // Published Topics
  roscpp::Parameter<std::string> sampled_path_topic{ "~/Published_Topics/path_sampled", "path_sampled" };
  roscpp::Parameter<std::string> waypoint_topic{ "~/Published_Topics/waypoint", "waypoint" };

  // Parameters
  roscpp::Parameter<std::string> base_frameId{ "~/Parameters/base_frameId", "base_link" };
  roscpp::Parameter<std::string> map_frameId{ "~/Parameters/map_frameId", "map" };

  roscpp::Parameter<double> pose_update_duration{ "~/Parameters/pose_update_duration", 0.1 };

private:
  PathSampler path_sampler_;
  std::queue<Eigen::Vector2d> waypoint_queue_;
  TransformHandler transform_handler_;

  roscpp::Subscriber<nav_msgs::Path> global_path_subscriber{ path_topic.param(), &WaypointManager::doPathSampling,
                                                             this };

  roscpp::Timer pose_update_timer{ pose_update_duration.param(), &WaypointManager::doArrivalMonitoring, this };

  roscpp::Publisher<nav_msgs::Path> sampled_path_publisher{ sampled_path_topic.param() };
  roscpp::Publisher<geometry_msgs::PoseStamped> waypoint_publisher{ waypoint_topic.param() };
};

WaypointManager::WaypointManager()
{
  pose_update_timer.start();
}

void WaypointManager::doPathSampling(const nav_msgs::PathConstPtr& msg)
{
  std::vector<Eigen::Vector2d> path;
  WavefrontPlannerRosConverter::fromPathMsg(*msg, path);
  waypoint_queue_ = path_sampler_.sampleByDistance(path, 5.0);

  geometry_msgs::PoseStamped msg_waypoint;
  WavefrontPlannerRosConverter::toPoseMsg(waypoint_queue_.front(), msg_waypoint);
  waypoint_publisher.publish(msg_waypoint);
}

void WaypointManager::doArrivalMonitoring(const ros::TimerEvent& event)
{
  geometry_msgs::TransformStamped robot_in_mapFrame;
  if (!transform_handler_.getTransform(map_frameId.param(), base_frameId.param(), robot_in_mapFrame))
    return;
  Eigen::Vector2d robot_position(robot_in_mapFrame.transform.translation.x, robot_in_mapFrame.transform.translation.y);

  // 1. when arrived
  if ((robot_position - waypoint_queue_.front()).norm() < 1.0)  // arrival condition
  {
    waypoint_queue_.pop();
    geometry_msgs::PoseStamped msg_waypoint;
    WavefrontPlannerRosConverter::toPoseMsg(waypoint_queue_.front(), msg_waypoint);
    waypoint_publisher.publish(msg_waypoint);
  }

  // 2. specific time interval
}

}  // namespace ros

#endif