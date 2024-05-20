/*
 * CostmapDemo.h
 *
 *  Created on: May 5, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef COSTMAP_DEMO_H
#define COSTMAP_DEMO_H

#include <ros/ros.h>

#include <dijkstra_planner_core/dijkstra_planner_core.h>
#include <dijkstra_planner_msgs/DijkstraPlannerMsgs.h>
#include <dijkstra_planner_demos/WavePropagatorDemo.h>

class CostmapDemo
{
public:
  CostmapDemo();

  void search();

  void findPath(const ros::TimerEvent& event);

  void publish(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_priv_{ "~" };

  // Frame Ids
  std::string map_frame{ nh_priv_.param<std::string>("mapFrame", "map") };

  std::string map_img_dir_{ nh_priv_.param<std::string>("mapImgDirectory",
                                                        "/home/ikhyeon/ros/test_ws/src/dijkstra_planner/"
                                                        "dijkstra_planner_demos/map/maze.png") };

  // Planner Options
  bool enable_search_unknown_{ nh_priv_.param<bool>("propagateUnknownCell", false) };
  double grid_resolution_{ nh_priv_.param<double>("gridResolution", 0.1) };
  double safe_distance_{ nh_priv_.param<double>("safeDistance", 0.0) };
  double goal_x_{ nh_priv_.param<double>("goalX", 1.26) };
  double goal_y_{ nh_priv_.param<double>("goalY", 1.75) };
  double robot_x_{ nh_priv_.param<double>("robotX", 1.2) };
  double robot_y_{ nh_priv_.param<double>("robotY", 0.3) };

  // Duration
  int path_update_rate_{ nh_priv_.param<int>("pathPublishRate", 20) };

  // ROS

  ros::Timer path_generation_timer_{ nh_priv_.createTimer(path_update_rate_, &CostmapDemo::findPath, this,
                                                          false, false) };
  ros::Timer publish_timer_{ nh_priv_.createTimer(ros::Duration(1.0), &CostmapDemo::publish, this, false,
                                                  false) };
  ros::Publisher pub_costmap_{ nh_priv_.advertise<grid_map_msgs::GridMap>("costmap", 1) };
  ros::Publisher pub_occupancy_map_{ nh_priv_.advertise<nav_msgs::OccupancyGrid>("occupancy_map", 10, true) };
  ros::Publisher pub_path_{ nh_priv_.advertise<nav_msgs::Path>("path", 10) };
  ros::Publisher pub_robot_{ nh_priv_.advertise<geometry_msgs::PoseStamped>("pose/robot", 10, true) };
  ros::Publisher pub_goal_{ nh_priv_.advertise<geometry_msgs::PoseStamped>("pose/goal", 10, true) };

private:
  WavePropagatorDemo wave_propagator_;
  OccupancyMap occupancy_map_;
};

#endif  // COSTMAP_DEMO_H