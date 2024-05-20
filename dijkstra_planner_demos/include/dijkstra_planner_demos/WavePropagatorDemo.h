/*
 * WavePropagatorDemo.h
 *
 *  Created on: May 5, 2024
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef WAVE_PROPAGATOR_DEMO_H
#define WAVE_PROPAGATOR_DEMO_H

#include "dijkstra_planner_core/WavePropagator.h"
#include <ros/ros.h>
#include <grid_map_ros/GridMapRosConverter.hpp>

class WavePropagatorDemo : public WavePropagator
{
public:
  /// @brief The algorithm spreads a "wave" of values starting from the goal cell and expanding outwards, assigning each
  /// cell a value indicating its distance from the goal.
  /// @param start_position The position of a robot in 2D
  /// @param goal_position The position of a goal in 2D
  /// @return True if the wave is reached to the start position, False otherwise
  bool isValidsearch(const grid_map::Position& goal_position, const grid_map::Position& robot_position);

  // BFS search: It is nothing but a priority queue based algorithm implementation
  // See https://www.geeksforgeeks.org/dijkstras-shortest-path-algorithm-using-priority_queue-stl/
  bool wavePropagation(const grid_map::Index& robot_index, const grid_map::Index& goal_index, ros::Publisher& pub, double pub_speed);
};

#endif  // WAVE_PROPAGATOR_DEMO_H