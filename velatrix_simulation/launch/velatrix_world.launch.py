#!/usr/bin/env python3
# -*- coding: utf-8 -*-
''' 
*****************************************************************************************
*  Filename:       velatrix_world.launch.py
*  Description:    Launch Ignition Gazebo Fortress world
*  Modified by:    Gouri Shankar
*****************************************************************************************
'''

import os
from os.path import join
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    velatrix_simulation = get_package_share_directory("velatrix_simulation")
    
    world_file = LaunchConfiguration(
        "world_file", 
        default=join(velatrix_simulation, "worlds", "disaster_area.sdf")
    )

    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r'"])
        }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='velatrix_bridge',
        parameters=[{
            'config_file': os.path.join(velatrix_simulation, 'config', 'bridge.yaml'),
            'qos_overrides./tf_static.publisher.durability': 'transient_local',
            'use_sim_time': True
        }],
        output='screen'
    )


    return LaunchDescription([
        # # Set resource paths for Gazebo
        # AppendEnvironmentVariable(
        #     name="IGN_GAZEBO_RESOURCE_PATH",
        #     value=join(velatrix_simulation, "worlds")
        # ),
        # AppendEnvironmentVariable(
        #     name="IGN_GAZEBO_RESOURCE_PATH",
        #     value=join(velatrix_simulation, "models")
        # ),
        # Declare launch arguments
        DeclareLaunchArgument("world_file", default_value=world_file),
        
        # Launch Gazebo
        gz_sim,
        bridge
    ])