#include "occupancy_gridmap_msgs/OccupancyGridMapMsgs.h"

bool OccupancyGridMapMsgs::toOccupancyGridMsg(const OccupancyGridMap& occupancy_map, nav_msgs::OccupancyGrid& msg)
{
  if (!occupancy_map.hasValidOccupancyLayer())
    return false;
  
  float min, max;
  std::tie(min, max) = occupancy_map.getMinMaxOccupancyData();
  grid_map::GridMapRosConverter::toOccupancyGrid(occupancy_map, "occupancy", min, max, msg);
  return true;
}

bool OccupancyGridMapMsgs::fromOccupancyGridMsg(const nav_msgs::OccupancyGrid& msg, OccupancyGridMap& occupancy_map)
{
  return grid_map::GridMapRosConverter::fromOccupancyGrid(msg, "occupancy", occupancy_map);
}
