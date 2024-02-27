/*
 * OccupancyGridMapHelper.h
 *
 *  Created on: Nov 23, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#include "occupancy_gridmap_core/OccupancyGridMap.h"
#include <grid_map_cv/GridMapCvConverter.hpp>
#include <opencv2/imgcodecs.hpp>

class OccupancyGridMapHelper
{
public:
  /// @brief From OpenCV image, occupancy map is created based on the pixel value [0, 255]
  /// @param image OpenCV image, which is an input (Expect gray image)
  /// @param resolution User defined parameter to set grid size
  /// @param occupancy_map Output of the function
  /// @return
  static bool initializeFromImage(const cv::Mat& image, const double resolution, OccupancyGridMap& occupancy_map);

  /// @brief Convert continuous occupancy value (0~1) to Binary state [0 or 1]
  /// @param occupancy_map Occupancy map as an input
  /// @param free_thres Below this value will be set to 0 (Free)
  /// @param occupied_thres Above this value will be set to 1 (Occupied)
  static void applyBinaryThreshold(double free_thres, double occupied_thres, OccupancyGridMap& occupancy_map);

  /// @brief Returns Inflated occupancy map with desired safety radius
  /// @param map Occupancy map to be inflated
  /// @param inflation_radius Desired safety radius
  /// @return Inflated Occupancy Map
  static bool getInflatedMap(const OccupancyGridMap& map, double inflation_radius, OccupancyGridMap& map_with_inflation);
};
