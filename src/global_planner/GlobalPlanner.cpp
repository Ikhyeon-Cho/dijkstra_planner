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
  if (use_map_from_image.param())
    readOccupancyMapFromImage();

  map_visualization_timer.start();
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

  if (inflation_radius.param() > std::numeric_limits<double>::epsilon())
  {
    OccupancyGridMap inflated_map;
    OccupancyGridMapHelper::getInflatedMap(occupancy_map, inflation_radius.param(), inflated_map);
    wavefront_planner_.setMap(inflated_map);
  }
  else
  {
    wavefront_planner_.setMap(occupancy_map);
  }
}

void GlobalPlanner::findPath(const geometry_msgs::PoseStampedConstPtr& msg)
{
  // Set goal position in map
  if (!transform_handler_.doTransform(*msg, map_frameId.param(), goalPose_in_mapFrame_))
  {
    // ROS_WARN("Failed to transform goal pose from %s to %s", msg->header.frame_id.c_str(),
    return;
  }

  // Set robot position in map
  geometry_msgs::TransformStamped robotPose_in_mapFrame;
  if (!transform_handler_.getTransform(map_frameId.param(), base_frameId.param(), robotPose_in_mapFrame))
  {
    std::cout << "Failed to get transform from " << base_frameId.param() << " to " << map_frameId.param() << std::endl;
    // return;
  }

  nav_msgs::Path msg_path;
  if (!findPathAt(robotPose_in_mapFrame, goalPose_in_mapFrame_, msg_path))
    return;

  ROS_INFO("Path found. Visualize Planned path");
  global_path_publisher.publish(msg_path);

  path_update_timer.start();

  // for debug purpose
  grid_map_msgs::GridMap costmap_msg;
  grid_map::GridMapRosConverter::toMessage(*wavefront_planner_.getCostMap(), costmap_msg);
  costmap_msg.info.header.frame_id = map_frameId.param();
  costmap_publisher.publish(costmap_msg);
}

bool GlobalPlanner::findPathAt(const geometry_msgs::TransformStamped& robot_pose,
                               const geometry_msgs::PoseStamped& goal_pose, nav_msgs::Path& msg_path)
{
  // Set goal position
  Eigen::Vector2d goal_position;
  WavefrontPlannerRosConverter::fromPoseMsg(goal_pose.pose, goal_position);

  // Set robot position
  Eigen::Vector2d robot_position;
  robot_position << robot_pose.transform.translation.x, robot_pose.transform.translation.y;

  // 1. Wavefront propagation
  if (!wavefront_planner_.doWavefrontPropagationAt(robot_position, goal_position))
  {
    ROS_WARN("Could not find feasible Path from current pose to goal.");
    return false;
  }

  // 2. Retrieve path
  std::vector<Eigen::Vector2d> path;
  bool has_valid_path = wavefront_planner_.doPathTracing(robot_position, goal_position, path);
  WavefrontPlannerRosConverter::toPathMsg(path, msg_path);
  return has_valid_path;
}

void GlobalPlanner::updatePath(const ros::TimerEvent& event)
{
  // Set goal position
  Eigen::Vector2d goal_position;
  WavefrontPlannerRosConverter::fromPoseMsg(goalPose_in_mapFrame_.pose, goal_position);

  // Update robot position
  geometry_msgs::TransformStamped robot_in_mapFrame;
  if (!transform_handler_.getTransform(map_frameId.param(), base_frameId.param(), robot_in_mapFrame))
  {
    // cout?
    return;
  }
  Eigen::Vector2d updated_robotPosition;
  updated_robotPosition << robot_in_mapFrame.transform.translation.x, robot_in_mapFrame.transform.translation.y;

  // Update path
  std::vector<Eigen::Vector2d> path_updated;
  bool can_update_path = wavefront_planner_.doPathTracing(updated_robotPosition, goal_position, path_updated);
  if (can_update_path)  // 1. Path tracing is available from existing costmap
  {
    nav_msgs::Path msg_path_updated;
    WavefrontPlannerRosConverter::toPathMsg(path_updated, msg_path_updated);
    global_path_publisher.publish(msg_path_updated);
  }
  else  // 2. If path tracing fails, then do replanning
  {
    nav_msgs::Path msg_path_updated;
    if (!findPathAt(robot_in_mapFrame, goalPose_in_mapFrame_, msg_path_updated))
    {
      ROS_ERROR("Replanning failed.");
      return;  // 3. If replanning fails, then return
    }

    ROS_INFO("Replanning done. found path");
    global_path_publisher.publish(msg_path_updated);
  }
}

void GlobalPlanner::visualizeOccupancyMap(const ros::TimerEvent& event)
{
  nav_msgs::OccupancyGrid msg;
  OccupancyGridMapRosConverter::toOccupancyGridMsg(wavefront_planner_.getMap(), msg);
  occupancy_map_publisher.publish(msg);
}

}  // namespace ros
