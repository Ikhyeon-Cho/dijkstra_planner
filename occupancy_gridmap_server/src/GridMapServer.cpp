#include "occupancy_gridmap_server/GridMapServer.h"

GridMapServer::GridMapServer()
{
  // Read image
  cv::Mat image = cv::imread(image_path_);
  if (image.empty())
  {
    std::cout << "Failed to read image from " << image_path_ << std::endl;
  }

  // Create occupancy grid map from image
  if (!OccupancyGridMapHelper::initializeFromImage(image, grid_resolution_, occupancy_map_))
  {
    std::cout << "Failed to initialize from image" << std::endl;
  }

  // Apply binary threshold
  OccupancyGridMapHelper::applyBinaryThreshold(occupancy_free_threshold_, occupancy_occupied_threshold_,
                                               occupancy_map_);

  if (inflation_radius_ > std::numeric_limits<double>::epsilon())
  {
    OccupancyGridMapHelper::getInflatedMap(occupancy_map_, inflation_radius_, inflated_map_);
  }

  map_visualization_timer_.start();
}

void GridMapServer::visualizeOccupancyGridMap(const ros::TimerEvent& event)
{
  // Publish occupancy grid map
  nav_msgs::OccupancyGrid occupancy_gridmap_msg;
  OccupancyGridMapMsgs::toOccupancyGridMsg(occupancy_map_, occupancy_gridmap_msg);
  pub_occupancy_gridmap_.publish(occupancy_gridmap_msg);

  // Publish inflated occupancy grid map
  nav_msgs::OccupancyGrid inflated_gridmap_msg;
  OccupancyGridMapMsgs::toOccupancyGridMsg(inflated_map_, inflated_gridmap_msg);
  pub_inflated_gridmap_.publish(inflated_gridmap_msg);
}
