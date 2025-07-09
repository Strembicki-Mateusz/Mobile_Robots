# Mobile Robots – laboratory 5 – Occupancy grid map

## General overview

Next assignments (mapping, path planning and navigation) will be combined into a single control program in the final stage. The overall goal is to implement a controller that will simultaneously move the robot along the planned path, while mapping obstacles and replanning the path if necessary. Please keep this in mind when implementing a mapping application, as structuring it appropriately will make developing the next parts easier.

## Introduction

The goal of the assignment is to use a laser scanner to create an occupancy grid map of the robot's surroundings.

Before the task, you should recall the formulas for coordinate system transformations (e.g., from the robot's local coordinate system to global coordinates).

Use a logarithmic probability representation to derive the occupancy grid map and a conventional representation to visualize it.


## T1: LIDAR based map

1. Define occupation grid map as a two dimensional array, with each cell representing a square area of given size.
2. Use laser scanner (in a fixed robot position) to calculate occupancy probabilities of the cells on the map and visualize resulting map. It may be assumed that the laser beam has a zero width (is a line). The map should be updated in iterations with each new measurement (it will be required for the next assignments).
3. Enhance the algorithm to include robot position from odometry (data from the pose topic) and build a map during robot motion.

**Note 1** Consider in calculations that the scanner is mounted ahead of robot center

**Note 2** Map may be presented in matplotlib with a custom script or by publishing ROS `nav_msgs/OccupancyGrid` message displayed in `rviz2`.


**Note 3** For this assignment, the robot motion may be executed from a separate, independant script


## Extensions:

1. Implement two versions of map updates: beam and cell oriented.
2. [difficult] Use sonars as an alternative map creation sensor (in this case, use the simplest inverse sensor model with distance to the obstacle and the beam width). Compare maps of two sensors. 


## Report

The report should be submitted via email in PDF format and must include the following:

1. Brief description of the algorithms used and exemplary maps, in particular the inverse sensor model that was used.
2. Comments on the choice of parameters used in the algrithm (with a comment how they influence the result).
3. A map created in the laboratory.

The source code of the implemented algorithms should also be commited to github.

## Additional Information

###  Exemplary maps
   ![Laboratory maps](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/2022_23/map_example1.png)

Maps created by: Beata Berajter, Ada Weiss, Małgorzata Witka-Jeżewska

![Comparison of maps created with a LIDAR and sonars](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/2022_23/map_example2.png)

Map created by: Karolina Durasiewicz, Piotr Dziębowski, Michał Smreczak
   

