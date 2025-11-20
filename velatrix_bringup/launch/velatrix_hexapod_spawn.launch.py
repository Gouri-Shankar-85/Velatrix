#!/usr/bin/env python3
# -*- coding: utf-8 -*-
''' 
*****************************************************************************************
*  Filename:       velatrix_hexapod_spawn.launch.py
*  Description:    Spawn the hexpod robot in Gazebo simulation environment
*  Modified by:    Gouri Shankar
*****************************************************************************************
'''

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Package Directories
    pkg_velatrix_description = get_package_share_directory('velatrix_description')
    pkg_velatrix_bringup = get_package_share_directory('velatrix_bringup')
    
    # Paths to files
    urdf_file = os.path.join(pkg_velatrix_description, 'urdf', 'velatrix.urdf.xacro')
    initial_positions_file = os.path.join(pkg_velatrix_description, 'config', 'initial_positions.yaml')
    joint_limits_file = os.path.join(pkg_velatrix_description, 'config', 'joint_limits.yaml')
    # controllers_file = os.path.join(pkg_velatrix_bringup, 'config', 'hexapod_controller.yaml')
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    fake_sensor_commands = LaunchConfiguration('fake_sensor_commands')
    sim_gazebo = LaunchConfiguration('sim_gazebo')
    sim_ignition = LaunchConfiguration('sim_ignition')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_use_fake_hardware = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='false',
        description='Use fake hardware (mock) if true'
    )
    
    declare_fake_sensor_commands = DeclareLaunchArgument(
        'fake_sensor_commands',
        default_value='false',
        description='Fake sensor commands for mock hardware'
    )
    
    declare_sim_gazebo = DeclareLaunchArgument(
        'sim_gazebo',
        default_value='false',
        description='Use gazebo classic if true'
    )
    
    declare_sim_ignition = DeclareLaunchArgument(
        'sim_ignition',
        default_value='true',
        description='Use Ignition Gazebo if true'
    )
    
    # Process URDF
    robot_description_content = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' use_fake_hardware:=', use_fake_hardware,
            ' fake_sensor_commands:=', fake_sensor_commands,
            ' sim_gazebo:=', sim_gazebo,
            ' sim_ignition:=', sim_ignition,
            ' initial_positions_file:=', initial_positions_file,
            ' joint_limits_file:=', joint_limits_file,
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # ROS-Gazebo Bridge Node
    bridge = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="velatrix_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
                ],
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            )
        ]
    )
    
    # Spawn Entity (Robot in Gazebo)
    spawn_entity = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'velatrix',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.0',  # Spawn slightly above ground
                    '-Y', '0.0',
                ],
                output='screen'
            )
        ]
    )
    
    # Joint State Broadcaster Spawner
    joint_state_broadcaster_spawner = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_state_broadcaster',
                    '--controller-manager', '/controller_manager',
                    # '--param-file', PathJoinSubstitution([pkg_velatrix_bringup, "config", "hexapod_controller.yaml"])
                ],
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time}
                ]
            )
        ]
    )

    # Position Controller Spawner  
    joint_trajectory_controller_spawner = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=[
                    'joint_trajectory_controller',
                    '--controller-manager', '/controller_manager',
                    # '--param-file', PathJoinSubstitution([pkg_velatrix_bringup, "config", "hexapod_controller.yaml"])
                ],
                output='screen',
                parameters=[
                    {'use_sim_time': use_sim_time}
                ]
            )   
        ]
    )
    return LaunchDescription([
        declare_use_sim_time,
        declare_use_fake_hardware,
        declare_fake_sensor_commands,
        declare_sim_gazebo,
        declare_sim_ignition,
        robot_state_publisher_node,
        bridge,
        spawn_entity,
        joint_state_broadcaster_spawner,
        joint_trajectory_controller_spawner,
    ])