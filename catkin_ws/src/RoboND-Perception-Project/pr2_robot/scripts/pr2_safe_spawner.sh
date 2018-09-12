#! /bin/bash
# DONT ACTUALLY RUN THIS!
 # Build python-pcl
#https://github.com/strawlab/python-pcl
pip install cython==0.25.2
python setup.py build_ext -i
python setup.py install

# These are the different command sequences for different puproses
# rosdep install --from-paths src --ignore-src --rosdistro=kinetic -y

#This Fixes gazebo crash in VM
# VMware: vmw_ioctl_command error Invalid argument.
export SVGA_VGPU10=0



# Capture and training
roslaunch sensor_stick training.launch &
rosrun sensor_stick capture_features.py &
rosrun sensor_stick train_svm.py &

# The real thing
roslaunch sensor_stick robot_spawn.launch

# PR2 pp Demo
roslaunch pr2_robot pick_place_demo.launch & sleep 10
roslaunch pr2_moveit pr2_moveit.launch & sleep 20 &&
rosrun pr2_robot pr2_motion

# Main Proj3
roslaunch pr2_robot pick_place_project.launch & 
rosrun pr2_robot segmentation.py &
rosrun pr2_robot p3main.py
