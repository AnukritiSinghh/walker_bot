# walker_bot
[![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)](https://opensource.org/licenses/MIT)

A walker algorithm using ros2 and turtlebot waffli-pi to avoid obstacles.

## Dependencies

- **Ubuntu 12.04**
- **ROS2 Humble**  
- **Turtlebot3 ROS Package**

## Installation 
1. All turtlebot3 pakages install
    ```bash
    sudo apt install ros-humble-turtlebot3*
    ```
2. Humble and Gazebo ROS packages
    ```bash
    sudo apt install ros-humble-gazebo-ros-pkgs 
    ```
3. Set up gazebo model path 
    ```bash
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
    prefix turtlebot3_gazebo \
    `/share/turtlebot3_gazebo/models/
    ```
