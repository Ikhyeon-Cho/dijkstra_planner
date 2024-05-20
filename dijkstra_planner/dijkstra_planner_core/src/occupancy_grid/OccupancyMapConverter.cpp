#include "occupancy_grid/OccupancyMapConverter.h"

bool OccupancyMapConverter::toROSMsg(const OccupancyMap& occupancy_map, nav_msgs::OccupancyGrid& msg)
{
  if (!occupancy_map.hasValidOccupancyLayer())
    return false;

  float min, max;
  std::tie(min, max) = occupancy_map.getMinMaxOccupancyData();
  grid_map::GridMapRosConverter::toOccupancyGrid(occupancy_map, "occupancy", min, max, msg);
  return true;
}

bool OccupancyMapConverter::fromROSMsg(const nav_msgs::OccupancyGrid& msg, OccupancyMap& occupancy_map)
{
  auto success = grid_map::GridMapRosConverter::fromOccupancyGrid(msg, "occupancy", occupancy_map);

  if (!success)
    return false;

  // divide by 100 to convert from ROS occupancy (100) to OccupancyMap::OCCUPIED (1)
  occupancy_map.getOccupancyLayer() /= 100.0;
  return true;
}
