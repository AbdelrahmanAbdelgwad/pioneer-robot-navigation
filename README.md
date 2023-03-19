# Closed Loop Pioneer Control

This is a ROS package that implements a closed-loop controller for controlling a Pioneer robot in the CoppeliaSim environment.

## Requirements

- ROS (tested on ROS Melodic)
- CoppeliaSim

## Installation

1. Clone this repository into your workspace:

    ```
    cd ~/catkin_ws/src
    git clone https://github.com/<your-username>/closed_loop_pioneer.git
    ```

2. Build the package:

    ```
    cd ~/catkin_ws
    catkin_make
    ```

## Usage

1. Run roscore:

    ```
    roscore
    ```

2. Start CoppeliaSim and load the scene "pioneer_control.ttt" located in the `closed_loop_pioneer/simulation` folder.

3. Run the node `move_pioneer_to_goal.py`:

    ```
    rosrun closed_loop_pioneer move_pioneer_to_goal.py
    ```

4. Publish the goal you want using this command:

    ```
    rostopic pub /pioneer/goal geometry_msgs/Pose2D "x: 1.0 y: 1.0 theta: 0.0"
    ```
    
## Demo

To see a demo of the package in action, check out this [YouTube video](https://www.youtube.com/watch?v=qk6bnAc4sz4&ab_channel=AbdelrahmanAbdelgawad).

