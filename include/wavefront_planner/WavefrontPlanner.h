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
  /// @return
  const OccupancyGridMap& getMap() const;

  /// @brief The algorithm spreads a "wave" of values starting from the goal cell and expanding outwards, assigning each
  /// cell a value indicating its distance from the goal.
  /// @param start_position The position of a robot in 2D
  /// @param goal_position The position of a goal in 2D
  /// @param path The shortest path from a start position to a goal position
  /// @param search_unknown_area
  /// @return True if
  bool findWavefrontPath(const Eigen::Vector2d& start_position, const Eigen::Vector2d& goal_position,
                         std::vector<Eigen::Vector2d>& path, bool search_unknown_area);

private:
  std::tuple<bool, std::vector<Eigen::Vector2d>> wavefrontAlgorithm(const grid_map::Index& start_index,
                                                                    const grid_map::Index& goal_index,
                                                                    bool search_unknown_area);

  OccupancyGridMap occupancymap_;

  Costmap::Ptr costmapPtr_;  // used for search algorithm
  bool has_map_{ false };
};

#endif