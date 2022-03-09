# HallwayROS1.0

<div align=center>
<img src="https://user-images.githubusercontent.com/36269534/157357452-38999f68-f6a8-46b6-86c7-529d02370c69.png" width="700"/>
</div>
* The pedestrian simulation is extended from [Pedsim_ros](https://github.com/srl-freiburg/pedsim_ros). We complete it, providing three optional person drivers: data replay, extended social force model, and manual control.
* The navigation simulation is extended from [move_base](https://github.com/ros-planning/navigation), a powerful ROS navigation stack. We simplify some existing algorithms and propose a participant-game-based algorithm. They are all adapted to our simulation platform. 

## Table of Contents
* [Installation](#1-Installation)
* [Quick Start](#2-Quick-Start)

## 1. Installation
The project has been tested on Ubuntu 16.04 (ROS Kinetic) and 18.04 (ROS Melodic). We highly recommend using Ubuntu 18.04 since Ubuntu 16.04 will no longer be supported after April 2021. In the following we will take ROS Melodic version as the example. Please install some dependence firstly: 
```
$ sudo apt install ros-melodic-navigation
```
Then please install this project and build it: 
```
$ mkdir -p HallwayROS_ws/src
$ cd HallwayROS_ws/src
$ git clone https://github.com/Chris-Arvin/HallwayROS1.0.git
$ cd ..
$ catkin_make
```

## 2. Quick Start
Please open a terminal to open the pedestrian simulation: 
```
$ source HallwayROS_ws/devel/setup.bash
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=2 robot_mode:=2
```
And open another terminal to open the navigation simulation: 
```
$ source HallwayROS_ws/devel/setup.bash
$ roslaunch move_base_bridge move_base_bridge.launch
```
There will be an interface to help you control the pedestrians as well as the robot. And you can have a glance to this simulation platform
<div align=center>
<img src="https://user-images.githubusercontent.com/36269534/157365577-70856fbb-ad9f-442b-a94f-005d3d043136.png" width="700"/>
</div>


## 3. Introduction for Key Parameters and Key Topics
@param: person_mode:
* 0 if drive the pedestrian with data replay
* 1 if drive the pedestrian with extended social force model
* 2 if drive the pedestrian with manual control

@param: robot_mode:
* 0 if drive the robot with algorithms(baselines or your own algorithm) in the format of the plugin
* 1 if drive the robot with extended social force model
* 2 if drive the robot with manual control

@topic: /map: do not project the person into the costmap, only including the static obstacles

@topic: /map_with_people: project the person into the costmap, regarding the persons as dynamic obstacles

@topic: /persons: the state of the persons, including their pose and velocity


## 4. More Examples
Change the params "person_mode" or "robot_mode" in [Quick Start](#2-Quick-Start) to use the simulation platform variously:
#### Case1: drive the person with extended social force model
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=1 robot_mode:=2
```
#### Case2: drive the person with data replay
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=0 robot_mode:=2
```
note:
* We provide 125 recorded data to help you test your algorithm. 
* You can easily create your own recorded data according to [Instructions to DIY](#5-Instructions to DIY)
#### Case3: drive the robot with extended social force
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=2 robot_mode:=1
```
#### Case4: drive the robot with provided baseline algorithms(DWA, TEB, Bezier, Time-bounded-lattice, participant game)
```
$ roslaunch pedsim_simulator pedsim_simulator.launch person_mode:=2 robot_mode:=0
```
note: 
* You can change the baseline in **move_base.launch**, please commen out and uncomment some lines as guidance.
* Also, you can adapt your own algorithm in the format of the plugin. The details about the plugin can be found at [how to create a plugin](http://wiki.ros.org/pluginlib/Tutorials/Writing%20and%20Using%20a%20Simple%20Plugin)

## 5. Instructions to DIY
#### DIY for simulation environment
We allow users to build its simulation environment in the format of xxx.xml. A reference xml file can be found at **Example_env.xml**. 
Slam for the environment is not necessary, because we bridge the auto map-sending function. 
You can create obstacles in your .xml file like:
```
<obstacle x1="1" y1="10" x2="5" y2="20" type="line">
<obstacle x="5" y="5" xHalfLength="2" yHalfLength="1.5" type="rectangle">
```
And you can use your environment to change **scene_file** in **pedsim_simulator.launch**

#### Create a new recorded data
Firstly, you should open the environment according to [Quick Start](#2-Quick-Start).
Next, open another terminal to record the keyborad input:
```
$ rosbag record 
```



## 6. Contributors
* Qianyi Zhang  zhangqianyi@mail.nankai.edu.cn
* Yinuo Song
* Zhengxi Hu
* Shilei Cheng
* Jingtai Liu

## 7. Acknowledgement






** We are so sorry: to avoid duplicate checks and data leakage, the whole work will be released after the paper is reviewed **
