/*
 * WavefrontPlanner.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef WAVEFRONT_PLANNER_H
#define WAVEFRONT_PLANNER_H

#include <occupancyMap/OccupancyGridMap.h>

#include "wavefront_planner/Costmap.h"

class WavefrontPlanner
{
public:
  using Ptr = std::shared_ptr<WavefrontPlanner>;
  using Position2D = Eigen::Vector2d;

  /// @brief
  WavefrontPlanner();

  /// @brief
  /// @param map
  void setMap(const OccupancyGridMap& map);

  /// @brief 
  /// @param search_unknown_area 
  void setSearchUnknownAreaOption(bool search_unknown_area);

  /// @brief
  /// @return
  const OccupancyGridMap& getMap() const;

  /// @brief
  /// @return
  const Costmap::Ptr& getCostMap() const;

  /// @brief The algorithm spreads a "wave" of values starting from the goal cell and expanding outwards, assigning each
  /// cell a value indicating its distance from the goal.
  /// @param start_position The position of a robot in 2D
  /// @param goal_position The position of a goal in 2D
  /// @return True if the wave is reached to the start position, False otherwise
  bool doWavefrontPropagationAt(const Eigen::Vector2d& start_position, const Eigen::Vector2d& goal_position);

  bool doPathTracing(const Eigen::Vector2d& start_position, const Eigen::Vector2d& goal_position,
                     std::vector<Eigen::Vector2d>& path);

private:
  // Dijkstra search: It is nothing but a priority queue based algorithm implementation
  // See https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/
  bool wavefrontPropagation(const grid_map::Index& start_index, const grid_map::Index& goal_index);

  // Backtracking of search algorithm: After (goal -> robot) search, load saved sequence of (robot -> goal) path
  void pathTracing(const grid_map::Index& start_index, const grid_map::Index& goal_index,
                   std::vector<Eigen::Vector2d>& path);

  OccupancyGridMap occupancymap_;

  Costmap::Ptr costmapPtr_;  // used for search algorithm
  bool has_map_{ false };
  bool search_unknown_area_{ false };
};

#endif