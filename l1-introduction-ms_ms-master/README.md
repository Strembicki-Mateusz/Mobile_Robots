# Mobile Robots – laboratory 1 – Introduction

Note: It is mandatory to follow laboratory safety rules, especially when operating robots.

## Introduction

The goal of this assignment is to become familiar with the ROS environment in the laboratory, operate robots, and read robot sensor data using Python scripts.

The assignment can be completed using the standard ROS2 subscriber/publisher approach.

Each Pioneer robot is connected within its own namespace, named `pioneerX`, where `X` represents the robot's number.
You can list available topics and observe data transmitted in any topic with:
```
ros2 topic list

ros2 topic echo <topicname>
```
The format of data packets sent on a topic can be checked with:
```
ros2 topic info <topicname>

ros2 interface show <msgtype>
```

To get general information about the data transmitted on a topic, use:
```
ros2 topic hz <topicname>

ros2 topic bw <topicname>
```

## T1: Exploring Topics Related to Pioneer Robots

1. List all topics available in the running ROS2 system.
2. Identify all topics related to a single robot.
3. Check the message format of selected topics, observe the data being transmitted, and attempt to determine the meaning of the data.

## T2: Accessing data from the laser scanner

1. Check the  message format in the `scan` topic.
2. Modify and run sample script [code/](code/)`ros2_lidar_subscriber.py`, to read data from the Hokuyo laser scanner.

## T3: Data visualization

1. Use the code from the visualization example in [examples/](examples/) to plot data using matplotlib.
Complete the calculations indicated by "TODO" comments to run the example.
2. Convert sensor data to Cartesian coordinates and display them in a plot

## Report

A report in Markdown format should be committed in the [report/](report/) folder before the next class.
The report should include:

1. A description of a single topic published by a robot that was analyzed in T1, including details on data format, message components, frequency, and bandwidth.
2. The final code used in T3, along with example plots.