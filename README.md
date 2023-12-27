# Multi-dimensional RRT Planner (C++)



https://github.com/daviddorf2023/ros2rrt/assets/113081373/6f8e5829-7414-44c8-915c-48ec843377d8

## Authors and Contributors
Author: David Dorf

## Description
3D and 2D implementation of Rapid Exploring Random Tree algorithms for ROS 2 in C++. This is a work-in-progress project stemming off of my Python implementation: https://github.com/daviddorf2023/multidim_rrt_planner. Not every feature mentioned in this readme has been added yet. The main ROS 2 nodes are rrt2D and rrt3D, which can take user inputs and publish the nodes of the RRT and the path to the goal. The ROS 2 nodes are currently in development. The nodes can subscribe to topics for map data, marker obstacles, and can be launched with rrt2Dlaunch.xml and rrt3Dlaunch.xml, respectively.

## Installation
### Dependencies
* ROS 2 Iron
* Eigen 3.4.0
### Building
Clone the repository into your ROS 2 workspace and build with colcon build.

## Usage
### ROS 2 Nodes
#### rrt2D
rrt2D is a ROS 2 node that implements the RRT algorithm in 2D. It can be launched with the rrt2Dlaunch.xml file. The node subscribes to the /occupancy_grid topic for map data and the /obstacle_markers_2D topic for marker obstacles. The node publishes the nodes of the RRT to the /rrt_markers topic and the path to the goal to the /rrt_path topic.
#### rrt3D
rrt3D is a ROS 2 node that implements the RRT algorithm in 3D. It can be launched with the rrt3Dlaunch.xml file. The node subscribes to the /occupancy_grid topic for map data and the /obstacle_markers_3D topic for marker obstacles. The node publishes the nodes of the RRT to the /rrt_markers topic and the path to the goal to the /rrt_path topic.
