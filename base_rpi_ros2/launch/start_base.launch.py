#!/usr/bin/env python3
#
# 6.2.2021 ---> 11.5.2021
#
# ros2 launch start_base.launch.py
#
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    joy_teleop_parameters_file = os.path.join('config', 'joy_teleop.yaml')

    return LaunchDescription([
         # / joy1 / joy[sensor_msgs / msg / Joy]
         # / joy1 / joy / set_feedback[sensor_msgs / msg / JoyFeedback]
         # / joy_teleop1 / joy[sensor_msgs / msg / Joy]
        Node(
            package='joy',
            namespace='joy1',
            executable='joy_node',
            name='saitek_joy'
        ),
        #
        DeclareLaunchArgument('teleop_config', default_value=joy_teleop_parameters_file),
        DeclareLaunchArgument('cmd_vel', default_value='input_joy/cmd_vel'),
        Node(
            package='joy_teleop',
            namespace='joy_teleop_adapter',
            executable='joy_teleop',
            parameters=[LaunchConfiguration('teleop_config')],
        ),
        #
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
