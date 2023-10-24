#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file = os.path.join(
        get_package_share_directory("factory_sim_gazebo_world"),
        "worlds", "no_roof_small_warehouse", "no_roof_small_warehouse.world"
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file,
                              'verbose': "true",
                              'extra_gazebo_args': 'verbose'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            launch_arguments={'verbose': "true"}.items()
        ),
    ])







