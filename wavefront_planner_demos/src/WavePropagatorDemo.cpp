/*
 * WavePropagatorDemo.cpp
 *
 *  Created on: Sep 1, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "wavefront_planner_demos/WavePropagatorDemo.h"

bool WavePropagatorDemo::isValidsearch(const grid_map::Position& goal_position,
                                       const grid_map::Position& robot_position)
{
  auto& costmap_ = getCostMap();

  // Check Validity of start and goal position: should be inside the map
  if (!costmap_.isDefinedAt(goal_position))
  {
    std::cout << "\033[33m"
              << "[ WARN] [ WavePropagatorDemo]: Invalid Position. Goal is out of the map" << std::endl;
    return false;
  }

  if (!costmap_.isDefinedAt(robot_position))
  {
    std::cout << "\033[33m"
              << "[ WARN] [ WavePropagatorDemo]: Invalid Position. Robot is out of the map" << std::endl;
    return false;
  }

  auto goal_index = costmap_.getGridIndexFrom(goal_position);
  auto robot_index = costmap_.getGridIndexFrom(robot_position);

  // Check Validity of start and goal position: should not be in occupied cell (or also unknown cell)
  if (costmap_.isLethalAt(robot_index))
  {
    std::cout << "\033[31m"
              << "[WARN] [ WavePropagatorDemo]: Invalid position. Robot is not in the FreeSpace" << std::endl;
    return false;
  }

  if (costmap_.isLethalAt(goal_index))
  {
    std::cout << "\033[33m"
              << "[ WARN] [ WavePropagatorDemo]: Invalid position. Goal is not in the FreeSpace" << std::endl;
    return false;
  }

  return true;
}

bool WavePropagatorDemo::wavePropagation(const grid_map::Index& goal_index, const grid_map::Index& robot_index,
                                         ros::Publisher& pub, double pub_speed)
{
  auto& costmap_ = getCostMap();

  // clear() is faster than add(layer)
  costmap_.clear("cost_wave_expansion");
  costmap_.clear("position_x_log");
  costmap_.clear("position_y_log");
  auto& wave_expansion_cost_layer = costmap_["cost_wave_expansion"];  // search cost will be logged here
  auto& position_x_log_layer = costmap_["position_x_log"];            // searched history (x) will be logged here
  auto& position_y_log_layer = costmap_["position_y_log"];            // searched history (y) will be logged here

  std::priority_queue<CostMap::Cell> searched_cell_list;
  CostMap::Cell goal_cell(goal_index, 0.0f);  // starts with zero cost
  searched_cell_list.push(goal_cell);         // "wave" from the goal cell

  int counter = 0;
  while (!searched_cell_list.empty() && ros::ok())
  {
    // Termination condition of wavefromt propagation: If wave reached start cell, then quit search
    // add 2 grid margin condition to handle the robot's localization error
    if (hasReachedAt(robot_index + grid_map::Index(0, 0)) && hasReachedAt(robot_index + grid_map::Index(2, 0)) &&
        hasReachedAt(robot_index + grid_map::Index(0, 2)) && hasReachedAt(robot_index + grid_map::Index(-2, 0)) &&
        hasReachedAt(robot_index + grid_map::Index(0, -2)))
      return true;

    const auto searched_cell = searched_cell_list.top();
    const grid_map::Index& searched_cell_index = searched_cell.index;
    const float& searched_cell_cost = searched_cell.cost;
    searched_cell_list.pop();

    const auto searched_cell_position = costmap_.getPositionFrom(searched_cell_index);

    auto wave_expansion_iterator = costmap_.getSquareIterator(searched_cell_index, 1);  // search nearest grid cells
    for (wave_expansion_iterator; !wave_expansion_iterator.isPastEnd(); ++wave_expansion_iterator)
    {
      if (!costmap_.isDefinedAt(*wave_expansion_iterator))
        continue;

      if (costmap_.isLethalAt(*wave_expansion_iterator))
        continue;

      // Square iterator searches the already searched grid cell, but it is not needed to search again
      if ((*wave_expansion_iterator == searched_cell_index).all())
        continue;

      // alias
      const auto& wavefront_index = *wave_expansion_iterator;
      auto& wavefront_cost = wave_expansion_cost_layer(wavefront_index(0), wavefront_index(1));

      auto& position_x_log = position_x_log_layer(wavefront_index(0), wavefront_index(1));
      auto& position_y_log = position_y_log_layer(wavefront_index(0), wavefront_index(1));

      const auto wave_expansion_cost =
          costmap_.getL2Dist(searched_cell_index, wavefront_index);  // Euclidean distance: Weighted Graph

      if (!std::isfinite(wavefront_cost))  // never visited before
      {
        wavefront_cost = searched_cell_cost + wave_expansion_cost;
        searched_cell_list.push(CostMap::Cell(wavefront_index, wavefront_cost));  // add to next search list (with cost)

        position_x_log = searched_cell_position(0);  // save history to the table
        position_y_log = searched_cell_position(1);
      }
      else if (wavefront_cost > searched_cell_cost + wave_expansion_cost)  // override the cost when lower cost arises
      {
        wavefront_cost = searched_cell_cost + wave_expansion_cost;

        position_x_log = searched_cell_position(0);
        position_y_log = searched_cell_position(1);
      }
      counter++;
      if (counter % (int)(pub_speed * 100) == 0)
      {
        grid_map_msgs::GridMap msg;
        std::vector<std::string> layers = { "cost_lethal", "cost_wave_expansion", "cost_elevation" };
        costmap_.setBasicLayers({"cost_wave_expansion"});
        grid_map::GridMapRosConverter::toMessage(costmap_, layers, msg);
        msg.info.header.frame_id = "map";
        pub.publish(msg);
        ros::Rate(500).sleep();
      }
    }
  }

  // Could not escape while loop >> Cannot reach the start index >> No feasible path
  return false;
}

// Backtracking of search algorithm: After (goal -> robot) search, load saved sequence of (robot -> goal)