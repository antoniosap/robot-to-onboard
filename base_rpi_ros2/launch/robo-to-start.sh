#!/bin/bash

#
# 4.3.2021 - robo-to base launch
#

# ros 2 - 4.2.2021 / 11.6.2021
source /opt/ros/foxy/setup.bash
source /usr/share/colcon_cd/function/colcon_cd.sh
export _colcon_cd_root=~/workspace-ros2
export ROS_VERSION=2
export ROS_PYTHON_VERSION=3
export ROS_DISTRO=foxy
export ROS_DOMAIN_ID=50


# ros - 31.12.2018
#ros-env kinetic
#source activate ros-env
#source /opt/ros/kinetic/setup.bash
# solo il nodo locale
#export ROS_HOSTNAME=rosrobot
#export ROS_MASTER_URI=http://192.168.147.85:11311
#export ROS_IP=192.168.147.85
# anche con nodi remoti
#export ROS_HOSTNAME=aldebaran
#export ROS_MASTER_URI=http://rosrobot:11311
#
#my ros workspace
source $HOME/workspace-ros2/install/setup.bash
#

cd $HOME/workspace-ros2/src/robot-to-onboard/base_rpi_ros2/launch || exit
ros2 launch start_base.launch.py
