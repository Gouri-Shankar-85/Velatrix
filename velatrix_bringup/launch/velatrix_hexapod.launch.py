#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    # Package Directories
    pkg_velatrix_description = get_package_share_directory('velatrix_description')
    pkg_velatrix_simulation = get_package_share_directory('velatrix_simulation')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_velatrix_bringup = get_package_share_directory('velatrix_bringup')

    # Paths
    urdf_file = os.path.join(pkg_velatrix_description, 'urdf', 'velatrix.urdf.xacro')
    world_file = os.path.join(pkg_velatrix_simulation, 'worlds', 'empty.sdf')
    bridge_config = os.path.join(pkg_velatrix_simulation, 'config', 'bridge.yaml')
    controller_config = os.path.join(pkg_velatrix_bringup, 'config', 'velatrix_hexapod_controller.yaml')

    # Process the URDF file
    doc = xacro.process_file(urdf_file)
    robot_desc = doc.toprettyxml(indent='  ')
    
    params = {
        'robot_description': robot_desc,
        'use_sim_time': True
    }

    # Declare arguments
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to the world file'
    )

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ]),
        launch_arguments={
            'gz_args': [LaunchConfiguration('world'), ' -r']
        }.items()
    )

    # Spawn the robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'velatrix',
            '-z', '0.3'
        ],
        output='screen'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': bridge_config,
            'use_sim_time': True
        }],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )

    # Joint Trajectory Controller
    joint_trajectory_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller'],
        output='screen'
    )

    # RViz
    # rviz_config = os.path.join(pkg_velatrix_description, 'rviz', 'velatrix.rviz')
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     arguments=['-d', rviz_config],
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    return LaunchDescription([
        declare_world_arg,
        declare_use_sim_time,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        bridge,
        joint_state_broadcaster,
        joint_trajectory_controller,
        # rviz
    ])