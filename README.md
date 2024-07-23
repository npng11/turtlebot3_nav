# Turtlebot3 Burger Autonomous Navigation in Gazebo

## Description 

Demonstrates autonomous navigation for Turtlebot3 in Gazebo simulation


## Requirement

- Ubuntu 20.04 
- ROS1 Noetic
  > Please find https://wiki.ros.org/ja/noetic/Installation/Ubuntu for ROS Noetic installation guidelines.


## Installation

1. Install dependent ROS packages 
``` bash
$ sudo apt-get install ros-noetic-joy ros-noetic-teleop-twist-joy \
  ros-noetic-teleop-twist-keyboard ros-noetic-laser-proc \
  ros-noetic-rgbd-launch ros-noetic-rosserial-arduino \
  ros-noetic-rosserial-python ros-noetic-rosserial-client \
  ros-noetic-rosserial-msgs ros-noetic-amcl ros-noetic-map-server \
  ros-noetic-move-base ros-noetic-urdf ros-noetic-xacro \
  ros-noetic-compressed-image-transport ros-noetic-rqt* ros-noetic-rviz \
  ros-noetic-gmapping ros-noetic-navigation ros-noetic-interactive-markers
```

2. Install Turtlebot3 packages
``` bash
$ sudo apt install ros-noetic-dynamixel-sdk
$ sudo apt install ros-noetic-turtlebot3-msgs
$ sudo apt install ros-noetic-turtlebot3
```

3. Craete workspace, install simulation package
``` bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src/
$ git clone -b noetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ~/catkin_ws && catkin_make
```

4. Clone this package into the 'src' directory and build

``` bash
$ cd ~/catkin_ws/src
$ git clone -b noetic-devel https://github.com/npng11/turtlebot3_nav.git
$ cd ~/catkin_ws && catkin_make
```


## Launch

1. Launch the autonomous navagation

``` bash
$ cd ~/catkin_ws
$ source devel/setup.bash
$ export TURTLEBOT3_MODEL=burger
$ roslaunch turtlebot3_nav auto_nav.launch
```
>The robot should start navigate to a random goal coordinates.


## Appendix
1. https://wiki.ros.org/ja/noetic/Installation/Ubuntu
2. https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/
3. https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation
