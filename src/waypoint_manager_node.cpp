/*
 * waypoint_manager_node.cpp
 *
 *  Created on: Dec 12, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "waypoint_manager/WaypointManager.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_manager_node");
  ros::NodeHandle nh("~");

  ros::WaypointManager node;

  ros::Rate loop_rate(50);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}