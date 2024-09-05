# Interactive Planner ROS

This package implements an interactive path planner using ROS. It allows users to select start and goal points on a map in RViz, and automatically calculates and displays the path between these points.

## Prerequisites

- Ubuntu 20.04
- ROS Noetic
- catkin workspace

## Installation

1. If you haven't already, install ROS Noetic by following the [official ROS installation guide](http://wiki.ros.org/noetic/Installation/Ubuntu).

2. Set up a catkin workspace:
   ```bash
   mkdir -p ~/catkin_ws/src
   cd ~/catkin_ws/
   catkin_make
   ```

3. Clone this repository into your catkin workspace's src directory:
   ```bash
   cd ~/catkin_ws/src
   git clone https://github.com/scogna02/robotic_programming.git
   ```

4. Install dependencies:
   ```bash
   cd ~/catkin_ws
   rosdep install --from-paths src --ignore-src -r -y
   ```

5. Build the package:
   ```bash
   catkin_make
   ```

6. Source the setup file:
   ```bash
   source ~/catkin_ws/devel/setup.bash
   ```

## Usage

1. Launch the interactive planner and RViz:
   ```bash
   roslaunch interactive_planner planner.launch
   ```

2. In RViz:
   - Use the "2D Nav Goal" tool to set the start point (first click).
   - Use the "2D Nav Goal" tool again to set the goal point (second click).
   - The planner will automatically calculate and display the path between these two points.
   - Repeat to set new start and goal points and recalculate the path.

## Configuration

- The map image used by the planner can be changed by modifying the `image_path` parameter in the `planner.launch` file.


