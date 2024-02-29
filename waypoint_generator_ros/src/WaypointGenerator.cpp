/*
 * WaypointGenerator.cpp
 *
 *  Created on: Dec 12, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "waypoint_generator_ros/WaypointGenerator.h"

WaypointGenerator::WaypointGenerator()
{
  // Print Node Info
  std::cout << std::endl;
  ROS_INFO("%-65s %s", ("Subscribing to the path topic: " + path_topic_).c_str(), "[ WaypointGenerator]");
  std::cout << std::endl;
}

void WaypointGenerator::findWaypoint(const nav_msgs::PathConstPtr& msg)
{
  nav_msgs::Path path_baselink;
  auto [has_transform, map_to_base] = transform_handler_.getTransform(map_frame_, baselink_frame_);
  if (!has_transform)
    return;

  // Transform waypoints to the base_link frame
  for (const auto& waypoint_map : msg->poses)
  {
    geometry_msgs::PoseStamped waypoint_baselink;
    ros_utils::tf::doTransform(waypoint_map, waypoint_baselink, map_to_base);

    Eigen::Vector2d waypoint_position;
    WavefrontPlannerMsgs::fromPoseMsg(waypoint_baselink.pose, waypoint_position);

    if (waypoint_position.norm() > distance_from_robot_)
      break;

    path_baselink.poses.push_back(waypoint_baselink);
  }
  path_baselink.header = msg->header;
  path_baselink.header.frame_id = baselink_frame_;

  // // Set orientation of the waypoints
  // for (size_t i = 0; i < path_baselink.poses.size() - 1; i++)
  // {
  //   auto x1 = path_baselink.poses[i].pose.position.x;
  //   auto y1 = path_baselink.poses[i].pose.position.y;
  //   auto x2 = path_baselink.poses[i + 1].pose.position.x;
  //   auto y2 = path_baselink.poses[i + 1].pose.position.y;
  //   double yaw = std::atan2(y2 - y1, x2 - x1);

  //   auto quat = ros_utils::tf::getQuaternionMsgFrom(0, 0, yaw);
  //   path_baselink.poses[i].pose.orientation = quat;
  // }
  // path_baselink.poses.back().pose.orientation = path_baselink.poses[path_baselink.poses.size() - 2].pose.orientation;
  
  // Publish the last waypoint
  pub_waypoint_.publish(path_baselink.poses.back());
}