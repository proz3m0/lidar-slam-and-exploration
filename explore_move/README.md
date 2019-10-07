# explore_move

A package that facilitates the fast exploration of a maze based map by following a set of pre determined waypoints


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

1. Start the simulation on any maze type world with a turtlebot

```bash
roslaunch turtlebot3_gazebo turtlebot3_maze_world.launch
```

2. If OGMAP using slam is not made already either build map using gmapping or refer to slam-to-loc

3. Start turtlebot navigation ensuring the map file argument in the launch file is the appropriate name, if slam-to-loc is being used ensure that the map argument is the same in both launch files

```bash
roslaunch turtlebot3_naviagation turtlebot3_navigation.launch 
```

4. Run the node that will offer the service to calculate the path

5. Run explore_move ensuring you check arguments, if move_base is struggling to get to waypoints increase comp_x comp_y comp_yaw in launch config which will increase the comparison delta in which allows the next target position to be published

```bash
roslaunch explore_move explore_move.launch
```


