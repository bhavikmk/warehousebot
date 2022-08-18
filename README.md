## Autonomous mobile robot for warehouse applications

### Project overview

Autonomous mobile robots, or AMRs, are swarm robotic systems capable of performing tasks and moving around the warehouse without the need for human intervention. These autonomous robots can improve warehouse efficiency by factors and can reduce cost significantly. 

This project includes AMR capable of autonomously navigate around warehouse by dynamically avoiding obstacles in real time. 

### Problem statement

Development of perception, planning and control systems of autonomous mobile robots for warehouse using robot operating system (ROS 2 (foxy)).

## Methodology

Updating soon...

### Algorithms and Techniques

Updating soon...

## Implementation

Updating soon...

## Results

Updating soon...

### Usage

**Build packages**

* `colcon build --packages-select warehouse_robot_spawner_pkg`

* `colcon build --packages-select warehouse_robot_controller_pkg`

**Launch packages**

* `ros2 launch warehouse_robot_spawner_pkg gazebo_world.launch.py`

* `ros2 launch warehouse_robot_controller_pkg controller_estimator.launch.py`

### references

1. [Simulation of warehouse robot using ros foxy](https://automaticaddison.com/how-to-simulate-a-robot-using-gazebo-and-ros-2/)