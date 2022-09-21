# lsc_dr_planner

This package presents a decentralized deadlock-free multi-agent trajectory planning algorithm in obstacle-dense environments.
The details about this algorithm can be found at the following links.

- **Authors:** Jungwon Park, Inkyu Jang, and H. Jin Kim from [LARR](http://larr.snu.ac.kr/), Seoul National University
- **Paper:** [Extended version](https://arxiv.org/abs/2209.09447)
- **Video:** [Youtube](https://youtu.be/PqfdbVfSujA)

![alt text](images/thumbnail.gif)


## 1. Install
This work is implemented based on C++17. Tested in the ROS Melodic, Ubuntu 18.04

(1) Install ROS Melodic for Ubuntu 18.04 or ROS Noetic for Ubuntu 20.04 (See http://wiki.ros.org/ROS/Installation, desktop-full version is recommended)

(2) Install CPLEX (https://www.ibm.com/products/ilog-cplex-optimization-studio)

(3) Set ROS distro

- ROS Melodic
```
export ROS_DISTRO=melodic
```
- ROS Noetic
```
export ROS_DISTRO=noetic
```

(4) Install dependancies and clone packages
```
sudo apt-get install ros-$ROS_DISTRO-octomap
sudo apt-get install ros-$ROS_DISTRO-octomap-*
sudo apt-get install ros-$ROS_DISTRO-dynamic-edt-3d
```

(5) Before building packages, check CMAKELIST that CPLEX_PREFIX_DIR is indicating the intallation location. For instance, if CPLEX is installed in ```/opt/ibm/ILOG/CPLEX_Studio201```, then CPLEX_PREFIX_DIR should be:
```
set(CPLEX_PREFIX_DIR /opt/ibm/ILOG/CPLEX_Studio201)
```

(6) Build packages
```
cd ~/catkin_ws
catkin_make
source ~/catkin_ws/devel/setup.bash
```


## 2. Demo
```
source ~/catkin_ws/devel/setup.bash
roslaunch lsc_dr_planner simulation.launch
```
The simulation result will be saved at ```lsc_dr_planner/log```.

## 3. Configuration
You can configure the simulation setting at the launch, mission files.
- ```launch/simulation.launch```: Mission, octomap, parameters for algorithm
- ```missions/*.json```: Start, goal, dynamical limits of the agent, map size

See the comments in the ```launch/simulation.launch``` and ```missions/readme.txt``` file for more details

Note: If you want to generate the mission file automatically, then use the matlab script in ```matlab/mission_generator```

## 4. Acknowledgment
This work is implemented based on the following packages.

(1) PIBT (https://github.com/Kei18/mapf-IR)

(2) rapidjson (https://rapidjson.org/)

(3) openGJK (https://www.mattiamontanari.com/opengjk/)

(4) convhull_3d (https://github.com/leomccormack/convhull_3d)

## 5. Notes
(1) Grid based planner issue: This code is not fully tested with various maps. Grid based planner may not work with some maps.
