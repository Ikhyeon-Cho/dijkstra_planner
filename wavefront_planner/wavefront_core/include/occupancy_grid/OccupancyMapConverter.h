/*
 * OccupancyMapConverter.h
 *
 *  Created on: Nov 27, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef OCCUPANCY_MAP_CONVERTER_H
#define OCCUPANCY_MAP_CONVERTER_H

#include "occupancy_grid/OccupancyMap.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>

class OccupancyMapConverter
{
public:
  static bool toROSMsg(const OccupancyMap& occupancy_map, nav_msgs::OccupancyGrid& msg);

  static bool fromROSMsg(const nav_msgs::OccupancyGrid& msg, OccupancyMap& occupancy_map);
};

#endif  // OCCUPANCY_MAP_CONVERTER_H