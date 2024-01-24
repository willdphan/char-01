# CHAR 01

## Project Overview

CHAR-01 project encompasses the development of an autonomous navigation robot, underpinned by a machine learning model that achieves approximately 67% accuracy developed with python and C++.

It leverages the capabilities of ROS2 and Gazebo Ignition for a comprehensive and realistic simulation, allowing for the sophisticated navigation of the autonomously driven robot. This blend of software development, machine learning, and simulation showcases an innovative approach to robotics and autonomous systems.

## Key Files

- `char_01_model.sdf` - Model for CHAR 01 Robot.
- `char_01_world.sdf` - World Environment for CHAR 01.
- `lidar_node` - subscribes to LaserScan messages and writes the range readings to a file.
- `visualize.py` - visualizes LiDAR sensor data.
- `train_model.py` - ML workflow. Preprocesses and encodes a dataset, selects key features for training a Random Forest Classifier.
- `predict.py` - ROS2 node in Python for making predictions based on LiDAR data using a trained machine learning model.
- `control.cpp` - Node that listens for predictions and publishes corresponding Twist messages to control a robot.

## Clone Repo

cd into the correct workspace to insert repo and type in terminal:

    git clone https://github.com/willdphan/char-01.git

## Start Up

Build the packages.

    colcon build

If you're having trouble, delete previous build and install folders and ensure that you clean out the previous build artifacts. 

    colcon build --symlink-install
    colcon clean

Source the setup.bach file.

    source install/setup.bash

Run the `char_01.launch.py` file.

    ros2 launch ros2_pkg char_01.launch.py

If you get error below:

    libGL error: pci id for fd 39: 1ab8:0010, driver (null) pci id for fd 40: 1ab8:0010, driver (null)

You don't have more graphics capabilities so you can use the command below in terminal to force software rendering as an alternative for now.

    export LIBGL_ALWAYS_SOFTWARE=1

## What Can Be Improved?

- Experimenting with different models might yield more efficient results.
- Reinforcement Learning.
- Faster processing times

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.