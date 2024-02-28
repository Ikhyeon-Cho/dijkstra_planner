# Wavefront planner
This repository provides the `wavefront_planner` ROS Node, which creates a planned path for a mobile robot on the basis of a grid map. The planner assumes a circular robot and operates on a costmap to find a minimum cost plan from a start point to an end point in a grid. 

## Overview
The wavefront expension algorithm is a specialized [potential field](https://en.wikipedia.org/wiki/Motion_planning#Artificial_potential_fields) path planner with breadth-first search ([BFS](https://en.wikipedia.org/wiki/Breadth-first_search)) to avoid local minima. It uses a growing circle around the robot. The nearest neighbors are analyzed first and then the radius of the circle is extended to distant regions.

## 1. Installation
**Dependencies:** This software is built on the Robotic Operating System ([ROS](https://www.ros.org/)). We assume that the followings are installed.
- Ubuntu and ROS (Tested on [ROS Noetic](https://wiki.ros.org/noetic), Ubuntu 20.04)
- [gridmap_server](https://github.com/Ikhyeon-Cho/gridmap_server) (The package provides the map, along with occupancymap - costmap interface)

**Building:** In order to install the `wavefront_planner` package, clone the latest version from this repository into your workspace and compile the package using ROS.
  ```
  cd ~/your-ros-workspace/src
  git clone https://github.com/Ikhyeon-Cho/gridmap_server.git
  git clone https://github.com/Ikhyeon-Cho/wavefront_planner.git
  cd ..
  catkin build wavefront_planner
  ```

## 2. Basic Usage
1. Configure the parameters in `wavefront_planner_ros/launch/config/params.yaml`
2. Run the launch file:
  ```
  roslaunch wavefront_planner run.launch
  ```