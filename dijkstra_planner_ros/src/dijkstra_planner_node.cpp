/*
 * dijkstra_planner_node.cpp
 *
 *  Created on: Sep 1, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "dijkstra_planner/DijkstraPlanner.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "dijkstra_planner");
  DijkstraPlanner dijkstra_planner_node;

  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}