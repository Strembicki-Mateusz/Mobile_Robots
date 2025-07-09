# Mobile Robots – laboratory 3 – Beacon based localization


## Introduction

The goal of this task is to familiarize yourself with the properties of the robot's global localization. The source of localization data is the Decawave/Qorvo radio localization system, which provides the distance between the robot and stationary markers ([Decawave/Qorvo system](https://www.qorvo.com/products/p/MDEK1001)).

The localization system in the laboratory consists of four fixed markers with known coordinates (x, y, z) and a receiver placed on the robot. In each measurement cycle, the receiver measures the distance to the markers and calculates the position using internal localization algorithms. The laboratory setup allows access to raw data (a list of transmitter coordinates and distances to them) as well as internally calculated localization (coordinates and quality parameters).

During the task, the implemented custom localization algorithm will be compared with the internally calculated localization and odometry.

**Note:** When developing the algorithm, take into account that in some measurement cycles, the received list may contain fewer than four reference markers, which means that the distance reading for one or more markers failed.

**Documentation of localization modules**
- [Real Time Location Systems. An Introduction](https://kcir.pwr.edu.pl/~jjakubia/l15/docs/Application%20Note%20APS003.pdf)
- [Product Datasheet: DWM1001-DEV](https://kcir.pwr.edu.pl/~jjakubia/l15/docs/DWM1001-DEV%20Data%20Sheet.pdf)


# Preparation

Before the lab session, you should review the task description and auxiliary materials within the provided scope and prepare the following:
1. Review the formulas for determining position using the trilateration method.
2. Propose an algorithm for determining position using four stationary markers.
3. Propose a method for determining localization error.

# T1 - Data Reading

1. Analyze the format of messages returned by the module:
   - a) Localization determined by the system’s internal algorithm (topic `/pioneerX/tag_pose`).
   - b) Source data (topic `/pioneerX/tag_status`).
2. Implement a script that subscribes to and processes data from both topics.
3. Implement the determination of the robot's position in the global coordinate system.

# T2 - Evaluation of Localization Quality

1. Using the provided files, compare the localization determined by the implemented algorithm with the one presented in the topics `tag_pose` and `odom`.
2. Perform example test runs and compare all existing localization methods (in addition to the above, also include your custom odometry calculated from wheel positions and velocities).
3. The program’s operation can be verified directly using the robot or with the help of recorded rosbag files ([rosbag files](https://kcir.pwr.edu.pl/~jjakubia/l15/lab_localization/)).

# Report

The lab report should be a repository containing the developed code and a `.md` file with the following:

- Description of the used localization algorithm, including applied formulas.
- Localization results for the provided files.
- Comparison results from T2, along with commentary.


