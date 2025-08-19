#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    swarm_core_dir = get_package_share_directory('swarm_core')
    
    # Launch arguments
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='cluttered',
        description='Gazebo world to load (open, cluttered, formation)'
    )
    
    enable_gazebo_arg = DeclareLaunchArgument(
        'enable_gazebo',
        default_value='true',
        description='Enable Gazebo simulation'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz2 visualization'
    )
    
    enable_plotjuggler_arg = DeclareLaunchArgument(
        'enable_plotjuggler',
        default_value='false',
        description='Enable PlotJuggler for data visualization'
    )
    
    # Gazebo launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                os.environ.get('GAZEBO_LAUNCH_DIR', '/opt/ros/humble/share/gazebo_ros/launch'),
                'gazebo.launch.py'
            ])
        ]),
        condition=IfCondition(LaunchConfiguration('enable_gazebo')),
        launch_arguments={
            'world': PathJoinSubstitution([
                swarm_core_dir, 'worlds', LaunchConfiguration('world') + '.world'
            ]),
            'verbose': 'true'
        }.items()
    )
    
    # PX4 SITL instances (these would typically be launched via separate scripts)
    # For now, we'll assume they're running or will be started manually
    
    # Swarm Core Nodes
    coordinator_node = Node(
        package='swarm_core',
        executable='coordinator',
        name='coordinator',
        output='screen',
        parameters=[
            PathJoinSubstitution([swarm_core_dir, 'config', 'params.yaml'])
        ],
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    avoidance_orca_node = Node(
        package='swarm_core',
        executable='avoidance_orca',
        name='avoidance_orca',
        output='screen',
        parameters=[
            PathJoinSubstitution([swarm_core_dir, 'config', 'params.yaml'])
        ],
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    coverage_node = Node(
        package='swarm_core',
        executable='coverage',
        name='coverage',
        output='screen',
        parameters=[
            PathJoinSubstitution([swarm_core_dir, 'config', 'params.yaml'])
        ],
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    # RViz2 visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d', PathJoinSubstitution([swarm_core_dir, 'config', 'swarm_visualization.rviz'])
        ],
        condition=IfCondition(LaunchConfiguration('enable_rviz'))
    )
    
    # PlotJuggler for data analysis
    plotjuggler_node = Node(
        package='plotjuggler',
        executable='plotjuggler',
        name='plotjuggler',
        arguments=[
            '--layout', PathJoinSubstitution([swarm_core_dir, 'config', 'swarm_analysis.xml'])
        ],
        condition=IfCondition(LaunchConfiguration('enable_plotjuggler'))
    )
    
    # Rosbag recording
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-a',
            '--duration', '600',  # 10 minutes
            '--output', 'swarm_run_' + LaunchConfiguration('world')
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_gazebo'))
    )
    
    return LaunchDescription([
        world_arg,
        enable_gazebo_arg,
        enable_rviz_arg,
        enable_plotjuggler_arg,
        
        # Launch order matters - Gazebo first, then nodes
        gazebo_launch,
        
        # Wait a bit for Gazebo to fully start
        ExecuteProcess(
            cmd=['sleep', '5'],
            output='screen'
        ),
        
        # Start swarm nodes
        coordinator_node,
        avoidance_orca_node,
        coverage_node,
        
        # Visualization tools
        rviz_node,
        plotjuggler_node,
        
        # Data recording
        rosbag_record,
    ])
