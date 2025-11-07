# Final Assignment

for the course: RO47003 - Robot Software Practicals 2025-2026, TU Delft  
Name: Hunor Adam Kovacs  
Date: 24/10/2025  

## Task Description

This project is the final lab assignement for the Robot Software Practicals, and consists of two ROS packages. The goal of the assignement is to drive the Mirte robot around a path marked with cones on the sides, and stop before the pedestrian, who is standing on the circular path.

## Setup

To run this project, after setting up the environment and sourcing your underlay first a workspace with a source directory is needed:
```bash
mkdir -p lab4_ws/src
```
```bash
cd lab4_ws/src
```
after navigating to the source directory as well, the mirte simulator as well as this repository need to be cloned:
```bash
git clone git@gitlab.ro47003.me.tudelft.nl:students-2526/ro47003_mirte_simulator.git
```
```bash
git clone git@gitlab.ro47003.me.tudelft.nl:students-2526/lab4/group74.git
```
after navigating out to the workspace directory, the project can be built, and the overlay can be sourced.
```bash
cd ..
```
```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
```
```bash
source install/setup.bash
```
## Running the code

The necessary simulations and files can be launched with this single launch file:
```bash
ros2 launch control_barrel_world solution.launch.xml
```
After the robot completed the track, and stopped in front of the pedestrian, the simulation can be stopped with a `CTRL+C` command from the terminal in which the simulation ran.

## Explanation of the packages

### pcl_obstacle_detector

The obstacle detection package is utilizing the structures and functions of the Point Cloud Library, to detect and segment the cones in the field of view of the robot. The topic /mirte/camera_depth provides a point cloud, which the node filters and separates into clusters, and produces 3D bounding boxes, which it publishes to the topic /detections in the form of a Detection3DArray.

### control_barrel_world

The other package has the job of controlling the robot. It subscribes to the above mentioned /detections topic, and further filters the bounding boxes, so it only registers the nearest ones. It also subscribes to the /pedestrians topic, where a node (which was provided as part of the assignement) publishes 2D bounding boxes from the camera feed, allowing the control node to detect if a pedestrian is too close, at which point it will stop. The core control logic is based on the cones detected around the robot: the position of the closest cone will make the robot move to the other direction, if there is no cone detected, the robot will move straight ahead.
