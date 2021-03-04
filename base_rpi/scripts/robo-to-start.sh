#!/bin/bash

#
# 4.3.2021 - robo-to base launch
#

# ros 2 - 4.2.2021
#source /opt/ros/foxy/setup.bash
#source /usr/share/colcon_cd/function/colcon_cd.sh
#export _colcon_cd_root=~/workspace-ros2
#export ROS_VERSION=2
#export ROS_PYTHON_VERSION=3
#export ROS_DISTRO=foxy
# ros 1 - 14.2.2021
source /opt/ros/noetic/setup.bash
export ROS_VERSION=1
export ROS_PYTHON_VERSION=3
export ROS_DISTRO=noetic

# ros - 31.12.2018
#ros-env kinetic
#source activate ros-env
#source /opt/ros/kinetic/setup.bash
# solo il nodo locale
export ROS_HOSTNAME=rosrobot
export ROS_MASTER_URI=http://192.168.147.85:11311
export ROS_IP=192.168.147.85
# anche con nodi remoti
#export ROS_HOSTNAME=aldebaran
#export ROS_MASTER_URI=http://rosrobot:11311
#
#my ros workspace
source /home/pi/workspace-ros1/devel/setup.bash
#

cd /home/pi/workspace-ros1/src/robot-to-onboard/base_rpi/launch || exit
/opt/ros/noetic/bin/roslaunch start_base.launch
