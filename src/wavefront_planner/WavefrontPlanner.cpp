/*
 * WavefrontPlanner.cpp
 *
 *  Created on: Sep 1, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "wavefront_planner/WavefrontPlanner.h"
#include <iostream>
#include "execution_timer/ExecutionTimer.h"

WavefrontPlanner::WavefrontPlanner()
{
}

void WavefrontPlanner::setMap(const OccupancyGridMap& map)
{
  occupancymap_.setGeometry(grid_map::Length(map.getLength()), map.getResolution());
  occupancymap_.getOccupancyLayer() = grid_map::GridMap::Matrix(map.getOccupancyLayer());
  has_map_ = true;

  costmapPtr_ = std::make_shared<Costmap>(map);
}

void WavefrontPlanner::setSearchUnknownAreaOption(bool search_unknown_area)
{
  search_unknown_area_ = search_unknown_area;
}

const OccupancyGridMap& WavefrontPlanner::getMap() const
{
  return occupancymap_;
}

const Costmap::Ptr& WavefrontPlanner::getCostMap() const
{
  return costmapPtr_;
}

bool WavefrontPlanner::doWavefrontPropagationAt(const Position2D& start_position, const Position2D& goal_position)
{
  // Check Validity of start and goal position: should be inside the map
  if (occupancymap_.isOutOfBoundaryAt(start_position) || occupancymap_.isOutOfBoundaryAt(goal_position))
    return false;

  auto start_index = occupancymap_.getIndexFrom(start_position);
  auto goal_index = occupancymap_.getIndexFrom(goal_position);

  // Check Validity of start and goal position: should not be in occupied cell
  if (occupancymap_.isOccupiedAt(start_index) || occupancymap_.isOccupiedAt(goal_index))
    return false;

  return wavefrontPropagation(start_index, goal_index);
}

bool WavefrontPlanner::wavefrontPropagation(const grid_map::Index& start_index, const grid_map::Index& goal_index)
{
  costmapPtr_->clearAll();                                           // faster than add layer
  auto& cost_layer = costmapPtr_->getCostLayer();                    // search cost will be logged here
  auto& position_x_log_layer = costmapPtr_->getLogPositionXLayer();  // searched history (x) will be logged here
  auto& position_y_log_layer = costmapPtr_->getLogPositionYLayer();  // searched history (y) will be logged here

  std::priority_queue<Costmap::CostComparator> search_list;
  float zero_cost{ 0.0f };
  search_list.push(Costmap::CostComparator(goal_index, zero_cost));  // "wave" from the goal cell

  while (!search_list.empty())
  {
    // Termination condition of while loop: If wave reached start cell, then quit search
    // add 2 margin to handle localization error
    if (costmapPtr_->isConnectedwithGoalAt(start_index) &&
        costmapPtr_->isConnectedwithGoalAt(start_index + grid_map::Index(2, 0)) &&
        costmapPtr_->isConnectedwithGoalAt(start_index + grid_map::Index(0, 2)) &&
        costmapPtr_->isConnectedwithGoalAt(start_index + grid_map::Index(-2, 0)) &&
        costmapPtr_->isConnectedwithGoalAt(start_index + grid_map::Index(0, -2)))
      return true;

    const auto queried_cell = search_list.top();
    const grid_map::Index& queried_index = queried_cell.index;
    const float& queried_index_cost = queried_cell.cost;
    search_list.pop();

    const auto queried_position = occupancymap_.getPositionFrom(queried_index);

    auto search_iterator = occupancymap_.getSquareIterator(queried_index, 1);  // search nearest grid cells
    for (search_iterator; !search_iterator.isPastEnd(); ++search_iterator)
    {
      if (occupancymap_.isOutOfBoundaryAt(*search_iterator))
        continue;

      if (occupancymap_.isOccupiedAt(*search_iterator))
        continue;

      if (!search_unknown_area_ && occupancymap_.isUnknownAt(*search_iterator))
        continue;

      // Square iterator searches center, but it is not needed (already searched)
      if ((*search_iterator == queried_index).all())
        continue;

      // alias
      const auto& search_index = *search_iterator;
      auto& wavefront_cost = cost_layer(search_index(0), search_index(1));

      auto& position_x_log = position_x_log_layer(search_index(0), search_index(1));
      auto& position_y_log = position_y_log_layer(search_index(0), search_index(1));

      const auto edge_cost = occupancymap_.getDistance(queried_index, search_index);  // weighted graph

      if (!std::isfinite(wavefront_cost))  // never visited before
      {
        wavefront_cost = queried_index_cost + edge_cost;
        search_list.push(Costmap::CostComparator(search_index, wavefront_cost));  // add to next search list (with cost)

        position_x_log = queried_position(0);  // save history to the table
        position_y_log = queried_position(1);
      }
      else if (wavefront_cost > queried_index_cost + edge_cost)  // override when lower cost arises
      {
        wavefront_cost = queried_index_cost + edge_cost;

        position_x_log = queried_position(0);
        position_y_log = queried_position(1);
      }
    }
  }

  // Could not escape while loop >> Cannot reach the start index: No feasible path
  return false;
}

bool WavefrontPlanner::doPathTracing(const Eigen::Vector2d& start_position, const Eigen::Vector2d& goal_position,
                                     std::vector<Eigen::Vector2d>& path)
{
  // Check Validity of start and goal position: should be inside the map
  if (occupancymap_.isOutOfBoundaryAt(start_position) || occupancymap_.isOutOfBoundaryAt(goal_position))
    return false;

  auto start_index = occupancymap_.getIndexFrom(start_position);
  auto goal_index = occupancymap_.getIndexFrom(goal_position);

  // Check Validity of start and goal position: should not be in occupied cell
  if (occupancymap_.isOccupiedAt(start_index) || occupancymap_.isOccupiedAt(goal_index))
    return false;

  if (!costmapPtr_->isConnectedwithGoalAt(start_index))
    return false;

  pathTracing(start_index, goal_index, path);
  return true;
}

void WavefrontPlanner::pathTracing(const grid_map::Index& start_index, const grid_map::Index& goal_index,
                                   std::vector<Eigen::Vector2d>& path)
{
  grid_map::Index index_backtracking(start_index);

  while (!(index_backtracking == goal_index).all())  // save (robot -> goal)
  {
    Position2D next_position(costmapPtr_->getLogPositionXLayer()(index_backtracking.x(), index_backtracking.y()),
                             costmapPtr_->getLogPositionYLayer()(index_backtracking.x(), index_backtracking.y()));

    path.push_back(next_position);

    index_backtracking = occupancymap_.getIndexFrom(next_position);
  }
  path.push_back(occupancymap_.getPositionFrom(goal_index));
}
// Backtracking of search algorithm: After (goal -> robot) search, load saved sequence of (robot -> goal)