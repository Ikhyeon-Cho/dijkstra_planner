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

#include <isr_ros_utils/core/core.h>
#include <isr_ros_utils/tools/TransformHandler.h>
#include <isr_ros_utils/tools/MsgConverter.h>

#include <occupancy_grid_map/OccupancyGridMapHelper.h>
#include <occupancy_grid_map/OccupancyGridMapRosConverter.h>

#include "wavefront_planner/WavefrontPlanner.h"
#include "wavefront_planner/WavefrontPlannerRosConverter.h"

namespace ros
{
class GlobalPlanner
{
public:
  GlobalPlanner();

  void readOccupancyMapFromImage();

  void visualizeOccupancyMap(const ros::TimerEvent& event);

  void updateTransform(const ros::TimerEvent& event);

  void goalCallback(const geometry_msgs::PoseStampedConstPtr& msg);

public:
  // ROS Parameters : Node
  roscpp::Parameter<std::string> goal_topic{ "GlobalPlannerNode/SubscribedTopic/goal", "/move_base_simple/goal" };
  roscpp::Parameter<std::string> occupancy_map_topic{ "GlobalPlannerNode/SubscribedTopic/occupancy_map", "/map" };
  roscpp::Parameter<std::string> global_path_topic{ "GlobalPlannerNode/PublishingTopic/path", "path" };

  // ROS Parameters : Framd Ids
  roscpp::Parameter<std::string> frameId_robot{ "frameId_robot", "base_link" };
  roscpp::Parameter<std::string> frameId_map{ "frameId_map", "map" };

  // Occupancy Map
  roscpp::Publisher<nav_msgs::OccupancyGrid> occupancy_map_publisher{ occupancy_map_topic };

  roscpp::Parameter<bool> read_map_from_image{ "GlobalPlannerNode/OccupancyMap/read_map_from_image", true };
  roscpp::Parameter<std::string> image_path{ "GlobalPlannerNode/OccupancyMap/image_path", "/home/isr" };
  roscpp::Parameter<double> grid_resolution{ "GlobalPlannerNode/OccupancyMap/grid_resolution", 0.1 };
  roscpp::Parameter<double> occupancy_free_threshold{ "GlobalPlannerNode/OccupancyMap/free_threshold", 0.3 };
  roscpp::Parameter<double> occupancy_occupied_threshold{ "GlobalPlannerNode/OccupancyMap/occupied_threshold", 0.7 };
  roscpp::Parameter<double> inflation_radius{ "GlobalPlannerNode/OccupancyMap/inflation_radius", 0.3 };
  roscpp::Parameter<double> visualization_duration{ "GlobalPlannerNode/OccupancyMap/visualize_duration", 1.0 };
  roscpp::Timer visualization_timer_occupancy{ visualization_duration.param(), &GlobalPlanner::visualizeOccupancyMap, this };

  // ROS Parameters: Planner
  roscpp::Subscriber<geometry_msgs::PoseStamped> goal_subscriber{ goal_topic.param(), &GlobalPlanner::goalCallback,
                                                                  this };
  roscpp::Publisher<nav_msgs::Path> global_path_publisher{ global_path_topic };

  roscpp::Parameter<bool> search_unknown_option{ "GlobalPlannerNode/Planner/search_unknown_option", false };

private:
  WavefrontPlanner wavefront_planner_;
  TransformHandler transform_handler_;
};
}  // namespace ros

#endif