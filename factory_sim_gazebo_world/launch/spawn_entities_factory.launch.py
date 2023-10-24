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


# At this stage this import isn't really usefull
# from moveit_configs_utils import MoveItConfigdBuilder

def generate_launch_description():
    # Define the gazebo node & launch the empty gazebo world - Gazebo empty world
    # gazebo = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([os.path.join(
    #     get_package_share_directory('gazebo_ros'),
    #     'launch', 'gazebo.launch.py')])      
    # )

    # Define the gazebo node & launch the empty gazebo world - Gazebo factory world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('factory_sim_gazebo_world'),
        'launch', 'factory_world_load.launch.py')])      
    )



    # Define the package path
    package_path = os.path.join(get_package_share_directory('factory_sim_gazebo_world'))

    # Define the XACRO file name and path (THIS MAY BE CHANGED - ONLY URDF ARE SUPPORTED (Hypo))
    xacro_file = os.path.join(get_package_share_directory('factory_sim_gazebo_world'), 'xacro', 'vehicle.urdf')

    # IDK maybe to make the hole thingy an xml urdf format I don't know
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description' : doc.toxml()}

    # Declare the robot state publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Spawn the entity inside the world
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'factory__'],
        output='screen'
    )
    return LaunchDescription(
        [gazebo,
        node_robot_state_publisher,
        spawn_entity]
    )
