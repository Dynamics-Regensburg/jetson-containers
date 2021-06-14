#!/bin/bash

source /opt/ros/noetic/setup.bash
mkdir -p ~/ros_catkin_ws/src/
cd ~/ros_catkin_ws/src/
git clone git://github.com/SICKAG/sick_scan.git
cd ..
catkin_make install
source ~/ros_catkin_ws/install/setup.bash

