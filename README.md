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

## Setting up

Before running the simulation, make sure we specify the model name of the turtlebot3 (burger, waffle or waffle_pi). Run the command below before roslaunch any of the launch file.

```bash
$ export TURTLEBOT3_MODEL=${TB3_MODEL}
```

If you want to permanently set the export settings follow the instruction below:

```bash
$ gedit ~/.bashrc
```

Add either one of the three selection into bashrc:
1. export TURTLEBOT3_MODEL=burger
2. export TURTLEBOT3_MODEL=waffle
3. export TURTLEBOT3_MODEL=waffle_pi

```bash
$ source ~/.bashrc
```

## Running the simulations

- Empty World

```bash
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
```

- TurtleBot3 World

```bash
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

- TurtleBot3 House

```bash
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

- To enable teleoperation on Gazebo

```bash
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

## Getting RVIS up

```bash
$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```


