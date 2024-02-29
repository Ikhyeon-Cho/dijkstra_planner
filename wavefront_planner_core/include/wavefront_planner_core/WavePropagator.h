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
#include <costmap_core/CostMap.h>

class WavePropagator
{
public:
  using Ptr = std::shared_ptr<WavePropagator>;
  using Position2D = Eigen::Vector2d;

  /// @brief
  WavePropagator();

  /// @brief
  /// @param map
  void initialize(const CostMap& costmap);

  /// @brief
  /// @return
  const CostMap& getCostMap() const;

  /// @brief The algorithm spreads a "wave" of values starting from the goal cell and expanding outwards, assigning each
  /// cell a value indicating its distance from the goal.
  /// @param start_position The position of a robot in 2D
  /// @param goal_position The position of a goal in 2D
  /// @return True if the wave is reached to the start position, False otherwise
  bool doWavePropagationAt(const grid_map::Position& goal_position, const grid_map::Position& robot_position);

  std::pair<bool, std::vector<grid_map::Position>> doPathGeneration(const grid_map::Position& robot_position,
                                                                    const grid_map::Position& goal_position);

private:
  // BFS search: It is nothing but a priority queue based algorithm implementation
  // See https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/
  bool wavePropagation(const grid_map::Index& robot_index, const grid_map::Index& goal_index);

  // Backtracking of searched path: After (goal -> robot) wavefront expansion, load logged sequence of (robot -> goal)
  void generatePath(const grid_map::Index& start_index, const grid_map::Index& goal_index,
                    std::vector<grid_map::Position>& path);

  bool hasWaveExpansionCostAt(const grid_map::Index& index);

  CostMap costmap_;
};

#endif  // WAVE_PROPAGATOR_H