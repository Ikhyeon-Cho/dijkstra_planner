/*
 * GlobalPlanner.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef GLOBAL_PLANNER_H
#define GLOBAL_PLANNER_H

#include <ros/ros.h>

#include <ros_node_utils/Core.h>
#include <ros_transform_utils/TransformHandler.h>

#include <occupancyMap/OccupancyGridMapHelper.h>
#include <occupancyMap/OccupancyGridMapRosConverter.h>

#include "wavefront_planner/WavefrontPlanner.h"
#include "wavefront_planner/WavefrontPlannerRosConverter.h"

namespace ros
{
class GlobalPlanner
{
public:
  GlobalPlanner();

  void readOccupancyMapFromImage();

  void findPath(const geometry_msgs::PoseStampedConstPtr& msg);

  bool findPathAt(const geometry_msgs::TransformStamped& robot_pose, const geometry_msgs::PoseStamped& goal_pose,
                  nav_msgs::Path& path);

  void visualizeOccupancyMap(const ros::TimerEvent& event);

  void updatePath(const ros::TimerEvent& event);

public:
  // Subscribed Topics
  roscpp::Parameter<std::string> goal_topic{ "~/Subscribed_Topics/goal", "/move_base_simple/goal" };
  roscpp::Parameter<std::string> occupancy_map_topic{ "~/Subscribed_Topics/occupancy_map", "/map" };

  // Published Topics
  roscpp::Parameter<std::string> global_path_topic{ "~/Published_Topics/global_path", "path" };
  roscpp::Parameter<std::string> inflated_map_topic{ "~/Published_Topics/inflated_map", "map_inflated" };

  // Parameters
  // -- Frame Ids
  roscpp::Parameter<std::string> base_frameId{ "~/Parameters/base_frameId", "base_link" };
  roscpp::Parameter<std::string> map_frameId{ "~/Parameters/map_frameId", "map" };
  // -- Occupancy Map
  roscpp::Parameter<bool> use_map_from_image{ "~/Parameters/use_map_from_image", true };
  roscpp::Parameter<std::string> image_path{ "~/Parameters/image_path", "/home/isr" };
  roscpp::Parameter<double> grid_resolution{ "~/Parameters/grid_resolution", 0.1 };
  roscpp::Parameter<double> occupancy_free_threshold{ "~/Parameters/occupancy_free_threshold", 0.3 };
  roscpp::Parameter<double> occupancy_occupied_threshold{ "~/Parameters/occupancy_occupied_threshold", 0.7 };
  roscpp::Parameter<double> inflation_radius{ "~/Parameters/inflation_radius", 0.3 };
  // -- Cycle Time
  roscpp::Parameter<double> path_update_duration{ "~/Parameters/path_update_duration", 0.05 };
  roscpp::Parameter<double> map_visualization_duration{ "~/Parameters/map_visualize_duration", 1.0 };
  // -- Wavefront Search
  roscpp::Parameter<bool> search_unknown_option{ "~/Parameters/search_unknown_option", false };

private:
  WavefrontPlanner wavefront_planner_;
  TransformHandler transform_handler_;
  geometry_msgs::PoseStamped goalPose_in_mapFrame_;

  roscpp::Subscriber<geometry_msgs::PoseStamped> goal_subscriber{ goal_topic.param(), &GlobalPlanner::findPath, this };
  roscpp::Publisher<nav_msgs::Path> global_path_publisher{ global_path_topic };
  roscpp::Publisher<nav_msgs::OccupancyGrid> occupancy_map_publisher{ inflated_map_topic.param() };

  roscpp::Timer path_update_timer{ path_update_duration.param(), &GlobalPlanner::updatePath, this };
  roscpp::Timer map_visualization_timer{ map_visualization_duration.param(), &GlobalPlanner::visualizeOccupancyMap,
                                         this };

  roscpp::Publisher<grid_map_msgs::GridMap> costmap_publisher{ "test" };
};
}  // namespace ros

#endif