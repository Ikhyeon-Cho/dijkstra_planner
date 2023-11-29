/*
 * GlobalPlanner.cpp
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "global_planner/GlobalPlanner.h"

namespace ros
{
GlobalPlanner::GlobalPlanner()
{
  if (read_map_from_image.param())
    readOccupancyMapFromImage();

  visualization_timer_occupancy.start();
}

void GlobalPlanner::readOccupancyMapFromImage()
{
  OccupancyGridMap occupancy_map;
  cv::Mat image = cv::imread(image_path.param());
  if (image.empty())
  {
    std::cout << "Failed to read image" << std::endl;
  }

  if (!OccupancyGridMapHelper::initializeFromImage(image, grid_resolution.param(), occupancy_map))
  {
    std::cout << "Failed to initialize from image" << std::endl;
  }

  OccupancyGridMapHelper::applyBinaryThreshold(occupancy_free_threshold.param(), occupancy_occupied_threshold.param(),
                                               occupancy_map);

  wavefront_planner_.setMap(occupancy_map);
}

void GlobalPlanner::visualizeOccupancyMap(const ros::TimerEvent& event)
{
  nav_msgs::OccupancyGrid msg;
  OccupancyGridMapRosConverter::toOccupancyGridMsg(wavefront_planner_.getMap(), msg);
  occupancy_map_publisher.publish(msg);
}

void GlobalPlanner::goalCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // Set goal position
  Eigen::Vector2d goal_position;
  WavefrontPlannerRosConverter::fromPoseMsg(msg->pose, goal_position);

  // Set robot position
  geometry_msgs::TransformStamped robot_in_mapFrame;
  transform_handler_.getTransform(frameId_map.param(), frameId_robot.param(), robot_in_mapFrame);
  Eigen::Vector2d robot_position(robot_in_mapFrame.transform.translation.x, robot_in_mapFrame.transform.translation.y);

  std::vector<Eigen::Vector2d> path;
  if (!wavefront_planner_.findWavefrontPath(robot_position, goal_position, path, search_unknown_option.param()))
  {
    ROS_WARN("Could not find feasible Path from current pose to goal.");
    return;
  }

  ROS_INFO("Path found. Visualize Planned path");
  nav_msgs::Path path_msg;
  WavefrontPlannerRosConverter::toPathMsg(path, path_msg);
  global_path_publisher.publish(path_msg);
}
}  // namespace ros
