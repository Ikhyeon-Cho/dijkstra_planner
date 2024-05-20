/*
 * CostMapConverter.h
 *
 *  Created on: Feb 29, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef COSTMAP_HELPER_H
#define COSTMAP_HELPER_H

#include "costmap/CostMap.h"

class CostMapConverter
{
public:
  static void fromOccupancyMap(const OccupancyMap& occupancy_map, CostMap& cost_map, bool unknown_is_lethal = true);
  static void fromInflationMap(const OccupancyMap& inflation_map, CostMap& cost_map, bool unknown_is_lethal = true);
};

#endif  // COSTMAP_HELPER_H