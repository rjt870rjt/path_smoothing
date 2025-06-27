# Path Smoothing & Trajectory Control in 2D Space (ROS 2)
![image](https://github.com/user-attachments/assets/a96ec264-9c6b-4ffe-850b-a31f13352c85)
See the simulation of path smoothing here: https://rjt870rjt.github.io/web_interface/

This project implements a complete navigation pipeline in ROS 2 for a differential drive robot (TurtleBot3), with the following core features:

- Path smoothing using Bézier curves or linear interpolation
- Trajectory generation with velocity profiling
- Trajectory tracking controller for differential drive motion
- Optional obstacle avoidance integration
- Visualization through RViz2
- Simulation environment in Gazebo

---

## Setup Instructions

### 1. Install Required Packages

Ensure that ROS 2 Humble is installed on your system.

Install TurtleBot3 Gazebo packages:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3-gazebo
```

```bash
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
source ~/.bashrc
```
### 2. Clone the Repository
- cd ~/ros2_ws/src
- git clone <https://github.com/rjt870rjt/path_smoothing/tree/main/src/path_smoothing_control>
- cd ~/ros2_ws
- colcon build --packages-select path_smoothing_control
- source install/setup.bash

### 3. Run the Simulation
```bash
ros2 launch path_smoothing_control turtlebot_path_tracking.launch.py
```
#### This will:
- Launch the TurtleBot3 simulation in Gazebo
- Initialize all nodes for waypoint publishing, path smoothing, trajectory generation and following
- Open RViz2 for real-time visualization    

## Project Architecture
#### The system is composed of the following modular nodes:
----
| Node                   | Description                                                     |
| ---------------------- | --------------------------------------------------------------- |
| `waypoint_publisher`   | Publishes 2D waypoints                                          |
| `path_smoother`        | Applies Bézier or linear interpolation for path smoothing       |
| `trajectory_generator` | Generates a time-indexed velocity profile                       |
| `trajectory_follower`  | Tracks the reference trajectory using a proportional controller |
| `obstacle_avoidance`   | Adds repulsive forces to avoid static obstacles (optional)      |
| `metrics_node`         | Tracks path-following error and completion percentage           |
--

## Design Choices and Algorithmic Details
### Why ROS 2 and TurtleBot3?

- ROS 2 provides real-time, modular communication and wide hardware support. TurtleBot3 offers easy integration with both simulation and real robots.
### Smoothing Algorithms
This project supports three different path smoothing techniques to convert discrete waypoints into a continuous path:

#### 1. Linear Interpolation
- Straight-line connections between waypoints
- Fast but results in sharp corners

#### 2. Bézier Curves
- Smooth polynomial curves
- Great for natural paths and smooth turning

#### 3. Cubic Splines
- Smooth interpolation ensuring continuous curvature and derivative
- Best suited for high-precision navigation tasks

### Velocity Profiling
The trajectory generator supports three types of velocity profiles, allowing the robot to move in a realistic and tunable manner:

#### 1. Constant Velocity
- Robot moves at a fixed speed
- Simple and easy to implement, but abrupt at start/stop

#### 2. Trapezoidal Velocity Profile
- ***Accelerates*** and decelerates linearly
- Smoothens start/stop without overshooting

#### 3. S-Curve Velocity Profile
- Velocity changes with smooth acceleration (jerk-limited)
- Ideal for physical robots to reduce wear and ensure safety

These can be selected dynamically using launch parameters or configuration settings:
```bash 
-- smoothing_algorithm bezier
-- velocity_profile trapezoidal
```
### Trajectory Control
- A simple proportional controller was used for both distance and heading.
- It is effective, easy to tune, and adequate for a 2D path tracking scenario.


## Obstacle Avoidance
### In Simulation
    - Static obstacles are modeled in Gazebo.
    - Repulsive force fields are computed based on obstacle proximity and added to the trajectory.
### In Real-Robot Scenarios
    - Use sensor_msgs/LaserScan from LiDAR to detect obstacles.
    - Convert laser scans into costmaps or dynamic points.
    - Implement avoidance using vector fields, dynamic window approach (DWA), or local planners.
    - Integrate with existing navigation stack if needed.

## How to Extend to a Real Robot
### Hardware Requirements

- A differential drive robot with onboard sensors (IMU, encoders, optional camera or LiDAR)
- Proper TF setup (map -> odom -> base_footprint)
- Localization stack (e.g., AMCL with a known map)

### Integration Steps
- Replace the Gazebo launch with the robot's bring-up launch file.
- Use real-time localization to compute current pose.
- Adjust controller gain parameters based on robot's physical constraints.
- Verify cmd_vel, tf, and odom topics are correctly mapped.

### The metrics_node calculates:
- Path error (distance from current robot pose to closest trajectory point)
- Completion percentage (how much of the trajectory has been executed)
- These metrics are printed in the terminal and can be plotted for analysis.

## Live Visualization

You can explore the interactive 2D path smoothing demo here:  
[🔗 View HTML Simulation](https://rjt870rjt.github.io/web_interface/)

## See the video demonstration here:
https://youtu.be/JrZuzASVuL8 

