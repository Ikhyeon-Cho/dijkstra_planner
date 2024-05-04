/*
 * OccupancyMap.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef OCCUPANCY_GRID_MAP_H
#define OCCUPANCY_GRID_MAP_H

#include <grid_map_core/grid_map_core.hpp>
#include <queue>
#include <limits>

class OccupancyMap : public grid_map::GridMap
{
  static constexpr float OCCUPIED = 1.0;
  static constexpr float FREE = 0.0;
  static constexpr float UNKNOWN = NAN;

public:
  OccupancyMap();

  grid_map::GridMap::Matrix& getOccupancyLayer();
  const grid_map::GridMap::Matrix& getOccupancyLayer() const;
  bool hasValidOccupancyLayer() const;

  std::tuple<float, float> getMinMaxOccupancyData() const;

  bool isValidAt(const grid_map::Index& grid_index);
  bool isValidAt(const grid_map::Position& position);

  bool isFreeAt(const grid_map::Index& grid_index) const;
  bool isOccupiedAt(const grid_map::Index& grid_index) const;
  bool isUnknownAt(const grid_map::Index& grid_index) const;

  bool isOutOfBoundaryAt(const grid_map::Index& grid_index) const;
  bool isOutOfBoundaryAt(const grid_map::Position& position) const;


  grid_map::Position getPositionFrom(const grid_map::Index& grid_index) const;
  grid_map::Index getIndexFrom(const grid_map::Position& grid_position) const;

  float getDistance(const grid_map::Index& grid_index1, const grid_map::Index& grid_index2) const;
  grid_map::CircleIterator getCircleIterator(const grid_map::Index& query_index, double radius) const;
  grid_map::SubmapIterator getSquareIterator(const grid_map::Index& query_index, int search_length) const;
};

#endif  // OCCUPANCY_GRID_MAP_H