#!/usr/bin/python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

import os
import sys
import rclpy
from gazebo_msgs.srv import SpawnEntity
import xacro


from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource

import xacro


def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    world_file = os.path.join(
        get_package_share_directory("aws_robomaker_small_warehouse_world"),
        "worlds", "no_roof_small_warehouse", "no_roof_small_warehouse.world"
    )
    xacro_file = os.path.join(get_package_share_directory('aws_robomaker_small_warehouse_world'), 'xacro', 'factory.urdf.xacro')
    assert os.path.exists(xacro_file), "The xacro doesnt exist in " + str(xacro_file)

    install_dir = get_package_prefix('aws_robomaker_small_warehouse_world')

    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] = \
            os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] = install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = \
            os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    spawn_car = Node(
        package="gazebo_ros",
        executable='spawn_entity.py',
        name = 'spawn_entity',
        arguments=['-entity', robot_desc, '-topic', '/robot_description'],
        output="screen"
        )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen")
    
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

    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        TimerAction(
            period=0.0,
            actions=[spawn_car]),
        TimerAction(
            period=0.0,
            actions=[robot_state_publisher])
    ])

    return ld