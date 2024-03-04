# Multi-dimensional RRT Planner (C++)
Author: David Dorf

## Description
3D and 2D implementation of Rapid Exploring Random Tree algorithms for ROS 2 in C++. This is a lightweight, performant version of my Python implementation: https://github.com/daviddorf2023/multidim_rrt_planner. Given a start and goal pose, the algorithm attempts to publish a path using the RRT algorithm when the `run_rrt` service is called. The main ROS 2 nodes are rrt2D and rrt3D, which can take user inputs and publish the nodes of the RRT and the path to the goal. The nodes can be launched with RViz by using `rrt2Dlaunch.xml` and `rrt3Dlaunch.xml`.

## Installation
Clone the repository into your ROS 2 workspace and build with `colcon build`, then source your install directory, and launch one of the launch files. More information can be found here: https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html.
