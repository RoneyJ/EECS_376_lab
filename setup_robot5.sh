#!/bin/bash
# Tyler Anderson
# Setup script for Glennan 210 Lab
# Connects to robot05

source /opt/ros/kinetic/setup.bash && echo "Sourcing kinetic"
cd catkin_ws && echo "Moving to catkin_ws"
source devel/setup.bash && echo "Sourcing devel"
export ROS_MASTER_URI=http://deeplearning05w.eecs.cwru.edu:11311 
echo "Setting up communications to robot05 for new terminal."

