# Mobile Robots – laboratory 6 – Path planning and navigation


## Introduction

The goal of the assignment is to use the occupancy grid map from the previous assignment for planning the path of the robot and navigation while avoiding obstacles.


## T1: Planning

Use a grid map to find a collision-free path from the current pose of the robot to a given destination.

### Elements to Consider in the Algorithm

1. **Proper Cell Size**: Ensure the cell size is appropriate for the planning algorithm.
2. **Map Transformation**: Convert the map into a format suitable for planning.
3. **Path Representation**: Define how the path will be represented.
4. **Validation Tests**: Conduct tests to validate the algorithm's correctness.
5. **Replanning Conditions**: Specify the conditions under which the path should be replanned.

### Wavefront Planner

#### Initialization
1. Set the distance value of the goal cell to `0`.
2. Assign a value of `+∞` to obstacle cells.
3. Mark the remaining cells as undefined.

#### Algorithm Steps
1. Initialize iteration number `i = 0`.
2. While there are undefined cells (or until the starting cell is reached):
   - Increment the iteration number `i` (current distance).
   - For each cell that is a neighbor of a cell with value `i-1`:
     - Set its value to the minimum value of its neighbors, increased by one (`i`).
3. **Path Selection**: Construct the final path from the starting cell to the goal by following a series of cells with decreasing values.

Refer to the explanation and [a visual example](http://www.societyofrobots.com/images/programming_wavefront_animation.gif) provided on the webpage [Society of Robots](http://www.societyofrobots.com/programming_wavefront.shtml).

## T2: Motion controller

Implement a robot controller that will follow the planned path.

The controller should:
- receive a path to follow (for example as a list of points to reach)
- send velocity commands (publishing `Twist` type messages to `/pioneerX/cmd_vel` topics) 

#### Points to Consider
- **Robot Movement**: How should the robot's movement be implemented?


## T3: Navigation with replanning

Integrate the mapping, planning and motion controller for collision free navigation of the robot.

The robot should:
- Receive the pose of a goal to navigate to.
- Plan a path to reach the goal in the free space (avoiding obstacles).
- Continuously avoid obstacles, update the map using current sensor readings, and re-plan the path if necessary as it moves.


#### Points to Consider
- **Obstacle Avoidance**: Is a separate obstacle avoidance behavior necessary?
- **Path Replanning**: How should the need for path replanning be determined?



## Report

The lab report should be a repository containing the developed code and a .md file with the following:

1. Brief description of the algorithms and assumptions used for planning and examples of planned paths.
2. Brief description of the navigation algorithm including motivation behind decisions.
3. Conclusions summarizing observations and improvement options.
The source code of the implemented algorithms should also be commited to github.

## Additional Information

### Moving the robot

1. Run safety system application `ros2 run safety_user_plugin user_plugin_node`
2. Choose your robot and enable motion (both buttons should be green)
3. You may send velocity commands to `/pioneerX/cmd_vel` topic

**Note:** You may control the robot with a keyboard with
`ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r __ns:=/pioneerX`


###  Control with a pad (joy)

1. Connect the device
2. Check device id with `ros2 run joy joy_enumerate_devices`
3. Run the driver node (note: to avoid name collisions with other groups, use namespace)
 `ros2 run joy joy_node --ros-args -r __ns:=/my_name_space`



