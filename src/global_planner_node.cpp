/*
 * wavefront_planner_node.cpp
 *
 *  Created on: Sep 1, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "wavefront_planner/WavefrontPlanner.h"
#include "global_planner/GlobalPlanner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wavefront_planner_node");
  ros::NodeHandle nh("~");

  ros::GlobalPlanner planner;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}