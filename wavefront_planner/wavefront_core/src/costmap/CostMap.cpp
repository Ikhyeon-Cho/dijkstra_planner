/*
 * CostMap.cpp
 *
 *  Created on: Dec 28, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "costmap/CostMap.h"

CostMap::CostMap() : grid_map::GridMap({ "cost_lethal", "cost_obstacle", "cost_inflation" })
{
}

CostMap::CostMap(const OccupancyMap& map) : grid_map::GridMap({ "cost_lethal", "cost_obstacle", "cost_inflation" })
{
  setGeometry(grid_map::Length(map.getLength()), map.getResolution());
  getLethalLayer() = map.getOccupancyLayer() * COST_LETHAL;
  // OccupancyLayer: NaN for unknown, 0.0 for free, 1.0 for occupied
}

grid_map::GridMap::Matrix& CostMap::getLethalLayer()
{
  return get("cost_lethal");
}

grid_map::GridMap::Matrix& CostMap::getObstacleLayer()
{
  return get("cost_obstacle");
}

grid_map::GridMap::Matrix& CostMap::getInflationLayer()
{
  return get("cost_inflation");
}

bool CostMap::isDefinedAt(const grid_map::Position& position)
{
  return isInside(position);
}

bool CostMap::isDefinedAt(const grid_map::Index& index)
{
  grid_map::Position pos;
  return getPosition(index, pos);
}

grid_map::Index CostMap::getGridIndexFrom(const grid_map::Position& position)
{
  grid_map::Index index;
  getIndex(position, index);
  return index;
}

grid_map::Position CostMap::getPositionFrom(const grid_map::Index& index)
{
  grid_map::Position position;
  getPosition(index, position);
  return position;
}

float CostMap::getL2Dist(const grid_map::Index& from, const grid_map::Index& to)
{
  grid_map::Position from_position, to_position;
  getPosition(from, from_position);
  getPosition(to, to_position);
  return (from_position - to_position).norm();
}

grid_map::CircleIterator CostMap::getCircleIterator(const grid_map::Index& query_index, double radius) const
{
  grid_map::Position queried_position;
  getPosition(query_index, queried_position);
  return grid_map::CircleIterator(*this, queried_position, radius);
}

grid_map::SubmapIterator CostMap::getSquareIterator(const grid_map::Index& query_index, int search_length) const
{
  grid_map::Index submap_start_index = query_index - grid_map::Index(search_length, search_length);
  grid_map::Index submap_buffer_size = grid_map::Index(2 * search_length + 1, 2 * search_length + 1);
  return grid_map::SubmapIterator(*this, submap_start_index, submap_buffer_size);
}

bool CostMap::isLethalAt(const grid_map::Index& index)
{
  const auto& cost_lethal = at("cost_lethal", index);
  return std::abs(cost_lethal - COST_LETHAL) < 1e-6;
}

bool CostMap::isFreeAt(const grid_map::Index& index)
{
  const auto& cost_lethal = at("cost_lethal", index);
  return std::abs(cost_lethal - COST_FREESPACE) < 1e-6;
}