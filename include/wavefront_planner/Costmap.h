/*
 * Costmap.h
 *
 *  Created on: Dec 28, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef WAVEFRONT_PLANNER_COSTMAP_H
#define WAVEFRONT_PLANNER_COSTMAP_H

#include <grid_map_core/grid_map_core.hpp>

class Costmap : public grid_map::GridMap
{
public:
  using Ptr = std::shared_ptr<Costmap>;

  Costmap() : grid_map::GridMap({ "cost", "position_x_log", "position_y_log" })
  {
  }

  Costmap(const OccupancyGridMap& map) : grid_map::GridMap({ "cost", "position_x_log", "position_y_log" })
  {
    setGeometryFrom(map);
  }

  void setGeometryFrom(const OccupancyGridMap& map)
  {
    setGeometry(grid_map::Length(map.getLength()), map.getResolution());
  }

  grid_map::GridMap::Matrix& getCostLayer()
  {
    return get("cost");
  }

  grid_map::GridMap::Matrix& getLogPositionXLayer()
  {
    return get("position_x_log");
  }

  grid_map::GridMap::Matrix& getLogPositionYLayer()
  {
    return get("position_y_log");
  }

  bool isEmptyAt(const grid_map::Index& index)
  {
    return !std::isfinite(at("cost", index));
  }

  bool isConnectedwithGoalAt(const grid_map::Index& index)
  {
    return std::isfinite(at("position_x_log", index));
  }

  // Grid Index with Cost
  struct CostComparator
  {
    grid_map::Index index;
    float cost{ -1 };

    CostComparator(grid_map::Index idx, float c) : index(idx), cost(c)
    {
    }
    bool operator<(const CostComparator& grid) const
    {
      return this->cost > grid.cost;
    }
  };
};

#endif