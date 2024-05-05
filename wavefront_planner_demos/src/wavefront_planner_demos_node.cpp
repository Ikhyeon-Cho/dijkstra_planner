/*
 * wavefront_planner_demo_node.cpp
 *
 *  Created on: May 5, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include <ros/ros.h>
#include "wavefront_planner_demos/WavefrontSearchDemo.h"
#include "wavefront_planner_demos/CostmapDemo.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wavefront_planner_demos");
  ros::NodeHandle nh;

  // Search demo
  bool search_demo{ false };
  nh.getParam("/wavefront_planner_demos/search_demo", search_demo);
  if (search_demo)
  {
    WavefrontSearchDemo search_demo;
    ros::spin();
  }

  bool costmap_demo{ false };
  nh.getParam("/wavefront_planner_demos/costmap_demo", costmap_demo);
  if (costmap_demo)
  {
    CostmapDemo costmap_demp;
    ros::spin();
  }

  return 0;
}