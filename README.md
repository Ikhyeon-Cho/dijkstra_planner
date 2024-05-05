# Wavefront planner
This repository provides the `wavefront_planner` ROS package, which creates a planned path for a mobile robot on the basis of an occupancy grid map. The planner finds a minimum cost plan from a start position to an end position, which assumes a circular robot. 

## Overview
The wavefront expansion algorithm is a specialized [potential field](https://en.wikipedia.org/wiki/Motion_planning#Artificial_potential_fields) path planner with breadth-first search ([BFS](https://en.wikipedia.org/wiki/Breadth-first_search)) to avoid local minima. It uses a growing circle around the robot. The nearest neighbors are analyzed first and then the radius of the circle is extended to distant regions.

Wave Propagation | Cost Map | Navigation
:---: | :---: | :---:
<img src="wavefront_planner/docs/wavefront_search.gif" width="200" /> | <img src="wavefront_planner/docs/costmap.gif" width="200" /> | <img src="wavefront_planner/docs/navigation.gif" width="200" /> |

## Installation
**Dependencies:** This software is built on the Robotic Operating System ([ROS](https://www.ros.org/)). We assume that the followings are installed.
- Ubuntu (Tested on 20.04) 
- ROS (Tested on [ROS Noetic](https://wiki.ros.org/noetic))

**Build:** In order to install the `wavefront_planner` package, clone the latest version from this repository and compile the package.
  ```
  cd ~/{your-ros-workspace}/src
  git clone https://github.com/Ikhyeon-Cho/wavefront_planner.git
  cd ..
  catkin build wavefront_planner
  ```

## 2. Basic Usage
1. Configure the parameters in `wavefront_planner_ros/config/params.yaml`
2. Run the launch file:
  ```
  roslaunch wavefront_planner run.launch
  ```

<p align='center'>
  <img src="wavefront_planner/docs/wavefront_planning.gif" width="500" />
</p>