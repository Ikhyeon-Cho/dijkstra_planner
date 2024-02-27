/*
 * GridMapServer.h
 *
 *  Created on: Feb 27, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef GRID_MAP_SERVER_H
#define GRID_MAP_SERVER_H

#include <ros/ros.h>

#include <occupancy_gridmap_core/OccupancyGridMap.h>
#include <occupancy_gridmap_core/OccupancyGridMapHelper.h>
#include <occupancy_gridmap_msgs/OccupancyGridMapMsgs.h>

// OpenCV - image reading
#include <opencv2/imgcodecs.hpp>

class GridMapServer
{
public:
  GridMapServer();

  void visualizeOccupancyGridMap(const ros::TimerEvent& event);

private:
  ros::NodeHandle nh_{ "~" };

  // Topics
  std::string occupancy_gridmap_topic_{ nh_.param<std::string>("occupancyGridMapTopic", "/map") };
  std::string inflated_gridmap_topic_{ nh_.param<std::string>("inflatedGridMapTopic", "map_inflated") };

  // Frame Ids
  std::string map_frame_{ nh_.param<std::string>("mapFrame", "map") };

  // Map parameters
  std::string image_path_{ nh_.param<std::string>("imagePath", "/home/ikhyeon") };
  double grid_resolution_{ nh_.param<double>("gridResolution", 0.1) };
  double occupancy_free_threshold_{ nh_.param<double>("freeThreshold", 0.3) };
  double occupancy_occupied_threshold_{ nh_.param<double>("occupiedThreshold", 0.7) };
  double inflation_radius_{ nh_.param<double>("inflationRadius", 0.3) };

  // Duration
  double map_visualization_rate_{ nh_.param<double>("publishRate", 1.0) };

  // ROS
  ros::Publisher pub_occupancy_gridmap_{ nh_.advertise<nav_msgs::OccupancyGrid>(occupancy_gridmap_topic_, 10) };
  ros::Publisher pub_inflated_gridmap_{ nh_.advertise<nav_msgs::OccupancyGrid>(inflated_gridmap_topic_, 10) };

  ros::Timer map_visualization_timer_{ nh_.createTimer(map_visualization_rate_, &GridMapServer::visualizeOccupancyGridMap,
                                                       this, false, false) };

private:
  OccupancyGridMap occupancy_map_;
  OccupancyGridMap inflated_map_;
};

#endif  // GRID_MAP_SERVER_H