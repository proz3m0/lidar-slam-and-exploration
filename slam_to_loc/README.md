# slam-to-loc

A package that allows seamless transition from teleoperated slam to autonomous naviagation with localisation


## Setting up

Before running the simulation, make sure we specify the model name of the turtlebot3 (burger, waffle or waffle_pi). Run the command below before roslaunch any of the launch file.

```bash
export TURTLEBOT3_MODEL=${TB3_MODEL}
```

If you want to permanently set the export settings follow the instruction below:

```bash
gedit ~/.bashrc
```

Add either one of the three selection into bashrc:
1. export TURTLEBOT3_MODEL=burger
2. export TURTLEBOT3_MODEL=waffle
3. export TURTLEBOT3_MODEL=waffle_pi

```bash
source ~/.bashrc
```

## Stage 1: Mapping using gmapping and teleoperation

1. Start the simulation (choose either one)

```bash
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch
roslaunch turtlebot3_gazebo turtlebot3_world.launch
roslaunch turtlebot3_gazebo turtlebot3_house.launch
```

2. Start teleop for turtle bot

- To enable teleoperation on Gazebo

```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

3. Start turtlebot gmapping

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch 
```

4. Map the entire world manually

5. Start the slam-to-loc and wait till pose saved comes up
```bash
roslaunch slam-to-loc slam-to-loc.launch
```

6. Start turtlebot navigation node ensuring that the map argument in the launch file is the same as the map argument in the slam-to-loc launch file
```bash
roslaunch turtlebot3_navigation turtlebot3_navigation.launch
```
The slam-to-loc node will have saved the map and publish the intital pose than shutdown, naviagation can now begin


