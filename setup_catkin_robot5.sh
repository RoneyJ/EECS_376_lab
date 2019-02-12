#!/bin/bash
# Tyler Anderson
# Setup script for Glennan 210 Lab
# Creates a catkin workspace and connects to robot02

source /opt/ros/kinetic/setup.bash

echo "Sourced ros kinetic"

cd ~

echo "Moved to home directory"

mkdir catkin_ws

echo "Made catkin_ws"

cd catkin_ws

echo "Moved to catkin_ws"

mkdir src

echo "Made src"

git config --global user.name "TAdragon1"
git config --global user.email "twa16@case.edu"

echo "Configured GitHub using Tyler's info"

#https://github.com/wsnewman/learning_ros_kinetic.git
#https://github.com/wsnewman/learning_ros_external_packages_kinetic.git
cd ~/catkin_ws/src && git clone https://github.com/RoneyJ/EECS_376_lab

echo "Cloned our lab GitHub"

cd ..

catkin_make

source devel/setup.bash

echo "Sourced devel/setup.bash"

echo "Setting up communications to robot05."

export ROS_MASTER_URI=http://deeplearning05w.eecs.cwru.edu:11311

ps aux | grep roscore

echo "Starting rviz"
  
rviz

# Cloning STDR is only necessary because of a bug in the current ROS binary release
# This prevents spawning a new robot on the screen. See this bug report for more information:
# https://github.com/stdr-simulator-ros-pkg/stdr_simulator/issues/195

# cd ~/ros_ws/src && git clone https://github.com/stdr-simulator-ros-pkg/stdr_simulator.git

# echo "export ROS_IP=`ifconfig eth0 2>/dev/null|awk '/inet addr:/ {print $2}'|sed 's/addr://'`" >> ~/.bashrc

