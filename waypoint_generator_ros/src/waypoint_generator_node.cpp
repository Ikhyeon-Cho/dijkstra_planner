/*
 * waypoint_generator_node.cpp
 *
 *  Created on: Dec 12, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "waypoint_generator_ros/WaypointGenerator.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoint_generator");
  WaypointGenerator waypoint_generator_node;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}