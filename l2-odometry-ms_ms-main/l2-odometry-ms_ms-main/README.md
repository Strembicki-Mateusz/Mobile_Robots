# Mobile Robots – laboratory 2 – Odometry


## Introduction

The goal of this assignment is to observe the properties of odometry in a robot localization task.
Before class, you should prepare by reviewing the documentation to find the dimensions of the robot (required for odometry calculations) and the formulas needed to compute odometry using the available data.

**Documentation of Pioneer robots:**
- [Pioneer 3DX specification](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/doc/Pioneer3DX-P3DX-RevA.pdf)
- [Pioneer 3 Operations Manual](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/doc/manual_pioneer.pdf)

## T1: Implementation

### 1. Verify Robot Dimensions
Measure and verify the robot’s dimensions, estimate the accuracy of these measurements, and assess their impact on odometry.

### 2. Implement Odometric Robot Localization
Develop a program to calculate the robot's position and orientation (x, y, θ) based on encoder data. Implement two versions: one using encoder ticks and another using velocities.
1. Use files containing [preprocessed datasets](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_data/) with predefined movements to develop the algorithm. [A code snippet for reading a CSV file](examples/snippet_csvread.py) is provided in the sample code section.
2. Write a subscriber to read encoder data from the robot (the `/pioneerX/joint_states` topic). Test your odometry calculations on the robot.

**Remarks:**
- **Data Format**: Verify the type and units of the messages in the topic; check for any data format issues.
- **Testing Order**: The suggested sequence for testing and tuning the algorithm with data files is as follows: forward/backward motion, right/left in-place rotation, and movement along a square trajectory.

### 3. Plan Experiments to Evaluate Odometry Error
1. Determine which factors you want to analyze for odometry error.
2. Design a trajectory for testing.
3. Conduct the experiment and collect results.


## Task 2 – Verification

In the verification part, use both the data collected from the robot during the experiments and the provided rosbags.
1. Compare the pose calculated with your script from Task 1 to the data received in the `/pioneerX/odom` topic.
2. Assuming the values read from the `/pioneerX/odom` topic represent the exact location of the robot, estimate the odometry error.

**!!Caution!!**: 
Do not replay the provided rosbags in the laboratory with the original topic names. Use the mapping option as follows:
```bash
ros2 bag play odom_backward -m /pioneer5/odom:=/bagodom /pioneer5/joint_states:=/bagjoint_states
```

**Remarks:**
- An example of obtaining pose from an odometry message can be found in [`examples/snippet_Odometry2Pose.py`](examples/snippet_Odometry2Pose.py).

## Report

A report in Markdown format should be committed in the [report/](report/) folder before the next class.
The report should include:

- Formulas for determining the robot’s pose based on positions and velocities, along with a list of parameters necessary for calculations.
- Estimated theoretical measurement and odometry errors.
- Observations and comments from Task 2.
- A summary of the verification process, detailing experiments used for tests and the evaluation of odometry error.

