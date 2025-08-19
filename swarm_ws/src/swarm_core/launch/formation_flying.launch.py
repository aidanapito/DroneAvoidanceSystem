#!/usr/bin/env python3
"""
Launch file for Formation Flying in Gazebo

This launch file starts:
- Gazebo simulation
- Formation controller node
- RViz for visualization
- Optional: Drone spawner
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    IncludeLaunchDescription,
    LogInfo
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution,
    PythonExpression
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for formation flying."""
    
    # Package directories
    pkg_share = get_package_share_directory('swarm_core')
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')
    
    # Launch arguments
    use_gazebo = LaunchConfiguration('use_gazebo')
    use_rviz = LaunchConfiguration('use_rviz')
    world_file = LaunchConfiguration('world_file')
    drone_count = LaunchConfiguration('drone_count')
    
    # Default values
    default_world = os.path.join(pkg_share, 'worlds', 'open.world')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_gazebo',
            default_value='true',
            description='Launch Gazebo simulation'
        ),
        
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for visualization'
        ),
        
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
        
        # Log launch configuration
        LogInfo(msg=[
            "🚁 Launching Formation Flying Test",
            "\n   Gazebo: ", use_gazebo,
            "\n   RViz: ", use_rviz,
            "\n   World: ", world_file,
            "\n   Drones: ", drone_count
        ]),
        
        # Launch Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    gazebo_pkg_share,
                    'launch',
                    'gazebo.launch.py'
                ])
            ]),
            condition=IfCondition(use_gazebo),
            launch_arguments={
                'world': world_file,
                'verbose': 'false',
                'pause': 'false'
            }.items()
        ),
        
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
            condition=IfCondition(use_rviz),
            arguments=[
                '-d', os.path.join(pkg_share, 'config', 'formation_flying.rviz')
            ]
        ),
        
        # Launch drone spawner (if needed)
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_drone1',
            output='screen',
            arguments=[
                '-entity', 'drone1',
                '-file', os.path.join(pkg_share, 'models', 'simple_drone', 'model.sdf'),
                '-x', '0.0',
                '-y', '0.0',
                '-z', '5.0'
            ],
            condition=IfCondition(use_gazebo)
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_drone2',
            output='screen',
            arguments=[
                '-entity', 'drone2',
                '-file', os.path.join(pkg_share, 'models', 'simple_drone', 'model.sdf'),
                '-x', '-3.0',
                '-y', '-3.0',
                '-z', '5.0'
            ],
            condition=IfCondition(use_gazebo)
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_drone3',
            output='screen',
            arguments=[
                '-entity', 'drone3',
                '-file', os.path.join(pkg_share, 'models', 'simple_drone', 'model.sdf'),
                '-x', '3.0',
                '-y', '-3.0',
                '-z', '5.0'
            ],
            condition=IfCondition(use_gazebo)
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
