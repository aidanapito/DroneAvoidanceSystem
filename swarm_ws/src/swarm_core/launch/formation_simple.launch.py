#!/usr/bin/env python3
"""
Simple Formation Flying Launch (No Gazebo Required)

This launch file starts just the formation controller
for testing formation logic without simulation dependencies.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    LogInfo
)
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for simple formation flying."""
    
    # Package directories
    pkg_share = get_package_share_directory('swarm_core')
    
    # Launch arguments
    drone_count = LaunchConfiguration('drone_count')
    use_rviz = LaunchConfiguration('use_rviz')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'drone_count',
            default_value='3',
            description='Number of drones in the swarm'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        
        # Log launch configuration
        LogInfo(msg=[
            "🚁 Launching Simple Formation Flying Test",
            "\n   Drones: ", drone_count,
            "\n   RViz: ", use_rviz,
            "\n   Note: Running without Gazebo simulation"
        ]),
        
        # Launch Formation Controller
        Node(
            package='swarm_core',
            executable='gazebo_formation_controller',
            name='formation_controller',
            output='screen',
            parameters=[{
                'drones.count': drone_count,
                'drones.names': ['drone1', 'drone2', 'drone3'],
                'formation.spacing': 3.0,
                'formation.altitude': 5.0,
                'formation.change_interval': 15.0,
                'formation.enable_auto_change': True,
                'control.rate': 20.0,
                'control.position_tolerance': 0.5,
                'control.max_velocity': 2.0
            }],
            remappings=[
                ('/formation/status', '/swarm/formation_status'),
                ('/formation/visualization', '/swarm/formation_viz')
            ]
        ),
        
        # Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', os.path.join(pkg_share, 'config', 'formation_flying.rviz')
            ]
        ),
        
        # Launch test script
        Node(
            package='swarm_core',
            executable='test_formation_commands',
            name='formation_test',
            output='screen',
            parameters=[{
                'test_interval': 5.0,
                'enable_auto_test': True
            }]
        )
    ])
