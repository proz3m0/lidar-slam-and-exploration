# lidar-slam-and-exploration

A turtlebot3 simulation ros packages to perform Simultaneous Localisation and Mapping (SLAM) of an unknown environment with frontier exploration and opencv object detection.

## Dependencies

This ROS package is created for Ubuntu Xenial (16.04) + ROS Kinetic Kame.

Please install catkin_tools to use 'catkin build' insteat of 'catkin make'

```bash
$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list'
$ wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
$ sudo apt-get update
$ sudo apt-get install python-catkin-tools
```

## Installation

```bash
$ mkdir -p ~/catkin_ws/src
$ cd catkin_ws/src
$ git clone https://github.com/UTSAnonymous/lidar-slam-and-exploration.git
$ cd ..
$ catkin build
$ source devel/setup.bash
```
