/*
 * WavePropagator.h
 *
 *  Created on: Sep 4, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef WAVE_PROPAGATOR_H
#define WAVE_PROPAGATOR_H

#include <occupancy_gridmap_core/OccupancyGridMap.h>
#include "wavefront_planner_core/Costmap.h"

class WavePropagator
{
public:
  using Ptr = std::shared_ptr<WavePropagator>;
  using Position2D = Eigen::Vector2d;

  /// @brief
  WavePropagator();

  /// @brief
  /// @param map
  void setGridSearchSpace(const OccupancyGridMap& map);

  /// @brief
  /// @param search_unknown_area
  void searchUnknownArea(bool search_unknown_area = false);

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
  bool doWavePropagationAt(const Eigen::Vector2d& goal_position, const Eigen::Vector2d& robot_position);

  bool doPathGeneration(const Eigen::Vector2d& robot_position, const Eigen::Vector2d& goal_position,
                     std::vector<Eigen::Vector2d>& path);

private:
  // Dijkstra search: It is nothing but a priority queue based algorithm implementation
  // See https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/
  bool wavefrontPropagation(const grid_map::Index& robot_index, const grid_map::Index& goal_index);

  // Backtracking of search algorithm: After (goal -> robot) search, load saved sequence of (robot -> goal) path
  void pathTracing(const grid_map::Index& start_index, const grid_map::Index& goal_index,
                   std::vector<Eigen::Vector2d>& path);

  OccupancyGridMap occupancymap_;

  Costmap::Ptr costmapPtr_;  // used for search algorithm
  bool search_unknown_area_{ false };
};

#endif  // WAVE_PROPAGATOR_H