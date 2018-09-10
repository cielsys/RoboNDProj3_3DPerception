#! /bin/bash
# DONT ACTUALLY RUN THIS!
# These are the different command sequences for different puproses

# PR2 pp Demo
roslaunch pr2_robot pick_place_demo.launch & sleep 10 &&
roslaunch pr2_moveit pr2_moveit.launch & sleep 20 &&
rosrun pr2_robot pr2_motion

# Main Proj3
roslaunch pr2_robot pick_place_project.launch & 
rosrun pr2_robot segmentation.py &
rosrun pr2_robot p3main.py
