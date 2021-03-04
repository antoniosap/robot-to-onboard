#!/bin/bash

#
# 4.3.2021 - robo-to base launch
#

cd /home/pi/workspace-ros1/src/robot-to-onboard/base_rpi/launch || exit
/opt/ros/noetic/bin/roslaunch start_base.launch
