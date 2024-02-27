/*
 * OccupancyGridMapHelper.h
 *
 *  Created on: Nov 27, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "occupancy_gridmap_core/OccupancyGridMap.h"
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <nav_msgs/OccupancyGrid.h>

class OccupancyGridMapMsgs
{
public:
static bool toOccupancyGridMsg(const OccupancyGridMap& occupancy_map, nav_msgs::OccupancyGrid& msg);

static bool fromOccupancyGridMsg(const nav_msgs::OccupancyGrid& msg, OccupancyGridMap& occupancy_map);
};