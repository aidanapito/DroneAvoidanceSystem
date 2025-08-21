#!/usr/bin/env python3
"""
Collision Avoidance Test Launch File

This launch file sets up a complete collision avoidance test environment:
- Gazebo world with moving obstacles
- Formation controller with collision avoidance
- Moving obstacles controller
- RViz visualization
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    LogInfo,
    ExecuteProcess
)
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for collision avoidance testing."""
    
    # Package directories
    pkg_share = get_package_share_directory('swarm_core')
    
    # Launch arguments
    world_file = LaunchConfiguration('world_file')
    drone_count = LaunchConfiguration('drone_count')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Default world file
    default_world = os.path.join(pkg_share, 'worlds', 'moving_obstacles.world')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'world_file',
            default_value=default_world,
            description='Gazebo world file to load'
        ),
        
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
            "🚧 Launching Collision Avoidance Test",
            "\n   World: ", world_file,
            "\n   Drones: ", drone_count,
            "\n   RViz: ", use_rviz,
            "\n   🚨 Moving obstacles will test collision avoidance!"
        ]),
        
        # Launch Gazebo in headless mode
        ExecuteProcess(
            cmd=['gazebo', '--headless', LaunchConfiguration('world_file')],
            output='screen'
        ),
        
        # Launch Formation Controller with Collision Avoidance
        Node(
            package='swarm_core',
            executable='gazebo_formation_controller',
            name='formation_controller',
            output='screen',
            parameters=[{
                'drones.count': drone_count,
                'drones.names': ['drone1', 'drone2', 'drone3'],
                'formation.spacing': 4.0,  # Increased spacing for safety
                'formation.altitude': 8.0,  # Higher altitude to avoid ground obstacles
                'formation.change_interval': 20.0,  # Slower changes for testing
                'formation.enable_auto_change': True,
                'control.rate': 30.0,  # Higher control rate for safety
                'control.position_tolerance': 0.3,
                'control.max_velocity': 1.5,  # Reduced speed for safety
                'collision_avoidance.enabled': True,
                'collision_avoidance.safety_distance': 2.0,
                'collision_avoidance.max_avoidance_velocity': 2.0
            }],
            remappings=[
                ('/formation/status', '/swarm/formation_status'),
                ('/formation/visualization', '/swarm/formation_viz')
            ]
        ),
        
        # Launch Moving Obstacles Controller
        Node(
            package='swarm_core',
            executable='moving_obstacles_controller',
            name='obstacles_controller',
            output='screen',
            parameters=[{
                'obstacle1.speed': 2.0,
                'obstacle2.speed': 1.5,
                'obstacle3.speed': 1.0
            }]
        ),
        
        # Launch RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d', os.path.join(pkg_share, 'config', 'collision_avoidance.rviz')
            ]
        ),
        
        # Launch test script for monitoring
        Node(
            package='swarm_core',
            executable='test_formation_commands',
            name='collision_test_monitor',
            output='screen',
            parameters=[{
                'test_interval': 10.0,
                'enable_auto_test': True,
                'monitor_collisions': True
            }]
        )
    ])
