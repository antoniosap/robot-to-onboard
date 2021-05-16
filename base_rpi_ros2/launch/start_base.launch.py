#!/usr/bin/env python3
#
# 6.2.2021 ---> 11.5.2021
#
# ros2 launch start_base.launch.py
#
import os
from pathlib import Path
from launch import LaunchDescription, logging
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    logger = logging.get_logger(__name__)
    cur_dir = os.path.dirname(os.path.abspath(__file__))

    # params_file = os.path.join(cur_dir, '..', 'config', 'params.yaml')
    # logger.info("params_file: " + params_file)

    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='saitek_joy',
            parameters=[{
                'dev': '/dev/input/js0',
                'deadzone': 0.3,
                'autorepeat_rate': 20.0,
            }]
        ),
        Node(
            package='teleop_twist_joy',
            name='teleop_twist_joy_node',
            executable='teleop_node',
            parameters=[{
                'axis_linear.x': 1,
                'axis_linear.y': 0,
                'scale_linear.x': 1.0,
                'scale_linear.y': 1.0,
                'axis_angular.yaw': 3,
                'scale_angular.yaw': 1.0,
                'enable_button': 0,
            }],
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim',
            remappings=[
                ('/turtlesim1/turtle1/cmd_vel', '/cmd_vel'),
            ],
        ),
    ])
