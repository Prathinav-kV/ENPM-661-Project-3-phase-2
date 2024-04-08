# ENPM-661-Project-3-phase-2

# Part 1

This document provides instructions for running a Python script designed for path planning and animation using the A* algorithm in a simulated environment with obstacles. The script calculates a path for a robot given start and goal positions, obstacles, and robot characteristics, then visualizes the path using pygame.

### Dependencies
To run this program, ensure you have the following dependencies installed:

Python 3.5 or newer

NumPy: For numerical calculations

Pygame: For visualization of the path and obstacles

A Python environment capable of executing scripts (e.g., command line, IDE)

Math, Datetime, queue, sys 

### How to Run

Open a terminal or command line interface.

Navigate to the directory containing the script.

Run the script by executing: python3 proj3p2_sarang_prathinav..py

The program will prompt you to enter several parameters, including clearance, start and goal positions, and wheel RPMs. Enter each value and press Enter.

### Example Input

Enter the following example set of inputs when prompted by the program:

Clearance: 2
Start X: 50
Start Y: 100
Start Theta: 0 (degrees)
Goal X: 575
Goal Y: 100
RPM1: 120
RPM2: 100

### Visualization
After calculating the path, the program will visualize the robot's movement from the start position to the goal position, navigating around the obstacles using pygame.


# Part 2
This README document provides instructions on how to run and launch the Turtlebot Close Loop Controller ROS Node, designed for navigating a Turtlebot through a predefined path in an closed-loop manner. The node, developed in Python, calculates the path based on inputs such as clearance, start and goal positions, and desired wheel RPMs.

### Dependencies
To run this program, you need to have the following libraries and ROS packages installed:

ROS 2 (Galactic)
Python 3.5 or newer
numpy for numerical operations
rclpy for ROS 2 Python clients
geometry_msgs and nav_msgs for ROS 2 message types
The path_finder.py that performs the A star algorithm to find the path

### How to Run
Ensure that ROS 2 is installed and sourced correctly on your system.

Navigate to your ROS 2 workspace's src folder and clone the project repository.

Build the workspace with colcon build and source the setup script (source install/setup.bash or source install/setup.zsh depending on your shell).

You need to have the path_finder.py available and it should provide a get_path function that returns a path based on your inputs.

Run the comeptition world with: ros2 launch turtlebot3_project3 competition.launch.py

Run the node with: ros2 run turtlebot3_project3 motion.py

### Example Input(all in cm)
When prompted by the program, enter the following example set of inputs:

Clearance: 2
Start X: 50
Start Y: 100
Start Theta: 0 degrees
Goal X: 575
Goal Y: 100
RPM1: 100
RPM2: 120

### Team Members
Prathinav Karnala Venkata
Directory ID: pratkv
UID: 120380983
Sarang Shibu
Directory ID: sarang
UID: 120254307




### Links:
Part01 animation - https://youtu.be/nR9b_wntx-I

Gazebo simulation video - https://youtu.be/lW22F28fQWw

Github link https://github.com/Prathinav-kV/ENPM-661-Project-3-phase-2

