/*
 * gridmap_server_node.cpp
 *
 *  Created on: Feb 27, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "occupancy_gridmap_server/GridMapServer.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "occupancy_gridmap_server");
  ros::NodeHandle nh("~");

  GridMapServer gridmap_server_node;
  ros::spin();

  return 0;
}