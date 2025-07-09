# Mobile Robots – laboratory 3 – Feature-based localization

## Introduction

The goal of this assignment is to extract features (line segments) from laser scans and use the extracted lines to determine robot position.

**Note**: Assignment for 2 laboratory classes. **Commit the state of the code at the end of each class**

## Preparation

Before class, you should review the task description and the auxiliary materials, covering the following:
- Coordinate Transformations:
  - Recall transformations of coordinates between polar and Cartesian systems.
  - Recall formulas for coordinate transformations between shifted and rotated coordinate systems relative to each other.
- Line detection methods:
  - Read the article [Nguyen, V. et al., "A comparison of line extraction algorithms using 2D laser rangefinder for indoor mobile robotics," IROS 2005, pp. 1929-1934](http://ieeexplore.ieee.org/xpls/abs_all.jsp?arnumber=1545234).
  - Choose a line detection algorithm to implement during class.


## T1: Line Detection Using a JSON File

1. Implement one of the selected line detection methods.
2. Adjust the parameters of the chosen method to optimize performance.
3. Using the provided data files, evaluate the effectiveness and correctness of your line detection implementation.

**Note**: To simplify the task, you can narrow the observed angle and limit the maximum distance of scanned data from the laser scanner.

## T2: Line-Based Localization – Using Data Files

Utilize the `line_localization_1.json` file and the code from the previous laboratory to calculate lines.

1. Define a global coordinate system (world frame).
2. Determine the position of the robot in the world frame.
3. Visually evaluate the localization results by comparing the calculated poses to the images provided below.
4. Determine the orientation of the robot in the world frame.

## T3: Evaluation of localization

1. Assume that in the first time step odometric (pose data in the odom frame) and feature-based poses match.
2. Calculate the transformation between the odom and world frames.
3. Calculate the error between the odometric and feature-based localization methods in the subsequent steps.

**Note**: To simplify the task, you can narrow the observed angle and limit the maximum distance of the scanned data.


## T4: Live Experiments (Optional)

1. Plan the Experiments: Design and conduct experiments using a robot.
2. Evaluate and analyze the experimental results.


## Report

The report should be submitted via email in PDF format and must include the following:

1. Brief description of the applied method: provide a concise explanation of the line detection algorithm used.
2. Present the results of the line detection process.
3. Present the results of localization for selected scenarios.
4. Evaluate the errors: visually and using calculations, compare with results from the previous assignments
5. (Optionally) Discuss results from the live experiments with robots.
6. Discuss your findings and provide conclusions based on the results.

## Additional Information

### Reference Data Structure

- **JSON File Structure**:
```json
  array { 
    object { 
      array { float(3) } pose  # [x, y, theta (in degrees)]
      float time
      array { float(512) } scan  # distances in meters in [-π/2, π/2]
    }  # a single measurement
  }
```

### Reference Data Folder

The reference data in JSON format is available at
(https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_localization_data/).
It contains:
1. [`line_detection_1`](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_localization_data/line_detection_1.json)

  ![Robot in a corner of the laboratory](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_localization_data/line_detection_1.jpg)
   
3. [`line_detection_2`](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_localization_data/line_detection_2.json)

   ![Robot in front of a box](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_localization_data/line_detection_2.jpg)
   
4. [`line_localization_1.json`](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_localization_data/line_localization_1.json) 

   ![Robot next to two walls - position 1](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_localization_data/line_localization_1a.jpg)
   ![Robot next to two walls - position 2](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_localization_data/line_localization_1b.jpg)
   ![Robot next to two walls - position 3](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_localization_data/line_localization_1c.jpg)
   ![Robot next to two walls - position 4](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_localization_data/line_localization_1d.jpg)
   ![Robot next to two walls - position 5](https://kcir.pwr.edu.pl/~jjakubia/MobileRobotics/lab2_localization_data/line_localization_1e.jpg) 

