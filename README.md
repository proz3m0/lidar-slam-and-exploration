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
## Getting RVIS up

```bash
$ roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
```

## Stage 1: Mapping using gmapping and teleoperation

1. Start the simulation (choose either one)

```bash
$ roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
$ roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

2. Start teleop for turtle bot

- To enable teleoperation on Gazebo

```bash
$ roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

3. Start turtlebot gmapping

```bash
$ roslaunch turtlebot3_slam turtlebot3_gmapping.launch
```

4. Map the entire world manually

5. Saving the map
```bash
$ rosrun map_server map_saver
```
This would save the map as an occupancy grid with 2 file (.pgm and .yaml)


