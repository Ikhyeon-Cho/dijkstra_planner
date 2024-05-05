/*
 * CostMapConverter.cpp
 *
 *  Created on: Feb 29, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "costmap/CostMapConverter.h"

void CostMapConverter::fromOccupancyMap(const OccupancyMap& occupancy_map, CostMap& cost_map, bool unknown_is_free)
{
  const auto& occupancy_layer = occupancy_map.getOccupancyLayer();
  auto& lethal_cost_layer = cost_map.getLethalLayer();

  cost_map.setGeometry(grid_map::Length(occupancy_map.getLength()), occupancy_map.getResolution(),
                       occupancy_map.getPosition());
  lethal_cost_layer = occupancy_layer * CostMap::COST_LETHAL;

  if (unknown_is_free)
  {
    // Nan to Free
    lethal_cost_layer = lethal_cost_layer.array().isNaN().select(CostMap::COST_FREESPACE, lethal_cost_layer);
  }
  else
  {
    // Nan to Lethal
    lethal_cost_layer = lethal_cost_layer.array().isNaN().select(CostMap::COST_LETHAL, lethal_cost_layer);
  }
}

void CostMapConverter::fromInflationMap(const OccupancyMap& inflation_map, CostMap& cost_map, bool unknown_is_free)
{
  const auto& occupancy_layer = inflation_map.getOccupancyLayer();
  auto& inflation_cost_layer = cost_map.getInflationLayer();

  inflation_cost_layer = occupancy_layer * CostMap::COST_LETHAL;

  if (unknown_is_free)
  {
    // Nan to Free
    inflation_cost_layer = inflation_cost_layer.array().isNaN().select(CostMap::COST_FREESPACE, inflation_cost_layer);
  }
  else
  {
    // Nan to Lethal
    inflation_cost_layer = inflation_cost_layer.array().isNaN().select(CostMap::COST_LETHAL, inflation_cost_layer);
  }
}