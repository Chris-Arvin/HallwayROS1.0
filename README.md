# HallwayROS1.0
* The pedestrian simulation is extended from [Pedsim_ros](https://github.com/srl-freiburg/pedsim_ros). We complete it, providing three optional person drivers: data replay, extended social force model, and manual control.
* The navigation simulation is extended from [move_base](https://github.com/ros-planning/navigation), a powerful ROS navigation stack. We simplify some existing algorithms and propose a participant-game-based algorithm. They are all adapted to our simulation platform. 


## Table of Contents
* [Installation](#1-Installation)
* [Quick Start](#2-Quick-Start)

## 1. Installation
The project has been tested on Ubuntu 16.04 (ROS Kinetic) and 18.04 (ROS Melodic). We highly recommend using Ubuntu 18.04 since Ubuntu 16.04 will no longer be supported after April 2021. In the following we will take ROS Melodic version as the example. Please install some dependence firstly: 
```

```


And please install this project and build it: 
```
$ mkdir -p HallwayROS_ws/src
$ cd HallwayROS_ws/src
$ git clone https://github.com/Chris-Arvin/HallwayROS1.0.git
$ cd ..
$ catkin_make
```

## 2. Quick Start
Please open terminal and open the pedestrian simulation: 
```
$ source HallwayROS_ws/devel/setup.bash
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=2 robot_mode:=2
```
And open another terminal and open the navigation simulation: 
```
$ source HallwayROS_ws/devel/setup.bash
$ roslaunch move_base_bridge move_base_bridge.launch
```
There will be an interface to help you control the pedestrians as well as the robot. And you can have a glance to this simulation platform


## 3. More Examples



## 4. Introduction for Key Parameters and Key Topics




## 5. Contributors
* Qianyi Zhang 
* Yinuo Song

## 6. Acknowledgement














The whole work will be released after the paper is accepted
