/*
 * PathSampler.h
 *
 *  Created on: Dec 12, 2023
 *      Author: Ikhyeon Cho
 *	 Institute: Korea Univ. ISR (Intelligent Systems & Robotics) Lab
 *       Email: tre0430@korea.ac.kr
 */

#ifndef PATH_SAMPLER_H
#define PATH_SAMPLER_H

#include <Eigen/Dense>
#include <queue>

class PathSampler
{
public:
  PathSampler();

  std::queue<Eigen::Vector2d> sampleByDistance(const std::vector<Eigen::Vector2d>& path, double sampling_distance);

private:
};

PathSampler::PathSampler()
{
}

std::queue<Eigen::Vector2d> PathSampler::sampleByDistance(const std::vector<Eigen::Vector2d>& path,
                                                           double sampling_distance)
{
  std::queue<Eigen::Vector2d> sampled_waypoints;
  if (path.empty())
    return sampled_waypoints;

  sampled_waypoints.push(path.front());
  Eigen::Vector2d current_goal(path.front());
  for (const auto& position : path)
  {
    if ((position - current_goal).norm() > sampling_distance)
    {
      sampled_waypoints.push(position);
      current_goal = position;
    }
  }
  sampled_waypoints.push(path.back());

  return sampled_waypoints;
}

#endif  // PATH_SAMPLER_H