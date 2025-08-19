#!/usr/bin/env python3
"""
Swarm Coordinator Node

Main coordination node for the multi-UAV swarm system.
Handles goal assignment, mission management, and high-level coordination.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup

import asyncio
import time
import threading
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

# ROS 2 messages
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from std_msgs.msg import String, Bool, Float32
from visualization_msgs.msg import MarkerArray, Marker

# Custom messages (if defined)
# from swarm_msgs.msg import SwarmStatus, CoverageStatus

# Local imports
from .utils.mavsdk_connector import SwarmConnector, MAVSDKConnector
from .utils.transforms import TransformUtils, SafetyUtils


class MissionState(Enum):
    """Enumeration of mission states."""
    IDLE = "idle"
    INITIALIZING = "initializing"
    TAKEOFF = "takeoff"
    COVERAGE = "coverage"
    FORMATION = "formation"
    LANDING = "landing"
    EMERGENCY = "emergency"
    COMPLETED = "completed"


@dataclass
class UAVGoal:
    """Data class for UAV goal information."""
    uav_id: str
    position: Point
    velocity: Optional[Twist] = None
    priority: int = 1
    timestamp: float = 0.0
    completed: bool = False


class SwarmCoordinator(Node):
    """
    Main swarm coordination node.
    
    Responsibilities:
    - Manage swarm state and mission execution
    - Assign goals to individual UAVs
    - Monitor safety and coordinate emergency procedures
    - Interface with MAVSDK for UAV control
    - Publish swarm status and mission information
    """
    
    def __init__(self):
        """Initialize the swarm coordinator node."""
        super().__init__('swarm_coordinator')
        
        # Initialize parameters
        self._init_parameters()
        
        # Initialize state
        self.mission_state = MissionState.IDLE
        self.uav_goals: Dict[str, UAVGoal] = {}
        self.mission_start_time = 0.0
        self.mission_timeout = 600.0  # 10 minutes
        
        # Initialize MAVSDK connector
        self.swarm_connector = SwarmConnector()
        self.mavsdk_thread = None
        self.mavsdk_loop = None
        
        # Initialize ROS 2 components
        self._init_ros_components()
        
        # Initialize mission parameters
        self._init_mission_params()
        
        # Start MAVSDK event loop in separate thread
        self._start_mavsdk_loop()
        
        # Start main control loop
        self.control_timer = self.create_timer(
            1.0 / self.get_parameter('control.offboard_rate').value,
            self._control_loop_callback
        )
        
        self.get_logger().info("Swarm Coordinator initialized")
    
    def _init_parameters(self):
        """Initialize node parameters from config file."""
        # UAV configuration
        self.declare_parameter('uav.count', 3)
        self.declare_parameter('uav.takeoff_height', 5.0)
        self.declare_parameter('uav.max_velocity', 3.0)
        self.declare_parameter('uav.safety_radius', 1.5)
        
        # Control parameters
        self.declare_parameter('control.offboard_rate', 50.0)
        self.declare_parameter('control.position_tolerance', 0.5)
        self.declare_parameter('control.velocity_tolerance', 0.1)
        self.declare_parameter('control.max_acceleration', 2.0)
        
        # Safety parameters
        self.declare_parameter('safety.min_separation', 1.0)
        self.declare_parameter('safety.emergency_stop_distance', 0.8)
        self.declare_parameter('safety.max_altitude', 15.0)
        
        # Mission parameters
        self.declare_parameter('mission.timeout', 600.0)
        self.declare_parameter('mission.enable_formation', True)
        self.declare_parameter('mission.formation_spacing', 3.0)
    
    def _init_ros_components(self):
        """Initialize ROS 2 publishers, subscribers, and services."""
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        fast_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Publishers
        self.swarm_status_pub = self.create_publisher(
            String, '/swarm/status', reliable_qos
        )
        
        self.mission_status_pub = self.create_publisher(
            String, '/swarm/mission_status', reliable_qos
        )
        
        self.uav_goals_pub = self.create_publisher(
            MarkerArray, '/swarm/uav_goals', fast_qos
        )
        
        self.emergency_pub = self.create_publisher(
            Bool, '/swarm/emergency', reliable_qos
        )
        
        # Subscribers
        self.odom_subscribers = {}
        self.coverage_map_sub = self.create_subscription(
            OccupancyGrid, '/swarm/coverage_map', 
            self._coverage_map_callback, fast_qos
        )
        
        # Services (if needed)
        # self.start_mission_srv = self.create_service(...)
        # self.stop_mission_srv = self.create_service(...)
        
        # Callback groups for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
    
    def _init_mission_params(self):
        """Initialize mission-specific parameters."""
        self.formation_positions = [
            Point(x=0.0, y=0.0, z=5.0),
            Point(x=3.0, y=0.0, z=5.0),
            Point(x=0.0, y=3.0, z=5.0)
        ]
        
        self.coverage_targets = []
        self.current_coverage_index = 0
    
    def _start_mavsdk_loop(self):
        """Start MAVSDK event loop in a separate thread."""
        def mavsdk_worker():
            self.mavsdk_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.mavsdk_loop)
            
            # Add UAVs to swarm
            for i in range(self.get_parameter('uav.count').value):
                uav_id = f"uav{i+1}"
                self.mavsdk_loop.run_until_complete(
                    self.swarm_connector.add_uav(uav_id)
                )
            
            # Start the event loop
            self.mavsdk_loop.run_forever()
        
        self.mavsdk_thread = threading.Thread(target=mavsdk_worker, daemon=True)
        self.mavsdk_thread.start()
    
    def _control_loop_callback(self):
        """Main control loop callback."""
        try:
            # Update mission state
            self._update_mission_state()
            
            # Execute current mission state
            self._execute_mission_state()
            
            # Check safety conditions
            self._check_safety_conditions()
            
            # Publish status updates
            self._publish_status_updates()
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
            self._handle_emergency()
    
    def _update_mission_state(self):
        """Update mission state based on current conditions."""
        current_time = time.time()
        
        # Check mission timeout
        if (self.mission_state != MissionState.IDLE and 
            current_time - self.mission_start_time > self.mission_timeout):
            self.mission_state = MissionState.COMPLETED
            self.get_logger().warn("Mission timeout reached")
            return
        
        # State transitions
        if self.mission_state == MissionState.IDLE:
            # Wait for mission start command
            pass
            
        elif self.mission_state == MissionState.INITIALIZING:
            # Check if all UAVs are connected and ready
            if self._all_uavs_ready():
                self.mission_state = MissionState.TAKEOFF
                self.get_logger().info("All UAVs ready, transitioning to takeoff")
            
        elif self.mission_state == MissionState.TAKEOFF:
            # Check if all UAVs have reached takeoff altitude
            if self._all_uavs_at_altitude():
                self.mission_state = MissionState.COVERAGE
                self.get_logger().info("Takeoff complete, starting coverage mission")
                
        elif self.mission_state == MissionState.COVERAGE:
            # Check if coverage is complete
            if self._coverage_complete():
                if self.get_parameter('mission.enable_formation').value:
                    self.mission_state = MissionState.FORMATION
                    self.get_logger().info("Coverage complete, transitioning to formation")
                else:
                    self.mission_state = MissionState.LANDING
                    self.get_logger().info("Coverage complete, starting landing sequence")
                    
        elif self.mission_state == MissionState.FORMATION:
            # Check if formation is stable
            if self._formation_stable():
                self.mission_state = MissionState.LANDING
                self.get_logger().info("Formation stable, starting landing sequence")
                
        elif self.mission_state == MissionState.LANDING:
            # Check if all UAVs have landed
            if self._all_uavs_landed():
                self.mission_state = MissionState.COMPLETED
                self.get_logger().info("Mission completed successfully")
    
    def _execute_mission_state(self):
        """Execute actions for the current mission state."""
        if self.mission_state == MissionState.INITIALIZING:
            self._execute_initialization()
            
        elif self.mission_state == MissionState.TAKEOFF:
            self._execute_takeoff()
            
        elif self.mission_state == MissionState.COVERAGE:
            self._execute_coverage()
            
        elif self.mission_state == MissionState.FORMATION:
            self._execute_formation()
            
        elif self.mission_state == MissionState.LANDING:
            self._execute_landing()
            
        elif self.mission_state == MissionState.EMERGENCY:
            self._execute_emergency()
    
    def _execute_initialization(self):
        """Execute initialization sequence."""
        # This would typically involve:
        # - Checking all UAV connections
        # - Verifying sensor data
        # - Setting up initial positions
        pass
    
    def _execute_takeoff(self):
        """Execute takeoff sequence."""
        # Send takeoff commands to all UAVs
        if self.mavsdk_loop and self.mavsdk_loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self.swarm_connector.takeoff_all(
                    self.get_parameter('uav.takeoff_height').value
                ),
                self.mavsdk_loop
            )
    
    def _execute_coverage(self):
        """Execute coverage mission."""
        # This will be coordinated with the coverage node
        # For now, just maintain hover positions
        pass
    
    def _execute_formation(self):
        """Execute formation flight."""
        # Assign formation positions to UAVs
        for i, uav_id in enumerate(self.swarm_connector.connectors.keys()):
            if i < len(self.formation_positions):
                goal = UAVGoal(
                    uav_id=uav_id,
                    position=self.formation_positions[i],
                    priority=1,
                    timestamp=time.time()
                )
                self.uav_goals[uav_id] = goal
    
    def _execute_landing(self):
        """Execute landing sequence."""
        # Send landing commands to all UAVs
        if self.mavsdk_loop and self.mavsdk_loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self.swarm_connector.land_all(),
                self.mavsdk_loop
            )
    
    def _execute_emergency(self):
        """Execute emergency procedures."""
        # Stop all UAVs immediately
        if self.mavsdk_loop and self.mavsdk_loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self.swarm_connector.emergency_stop_all(),
                self.mavsdk_loop
            )
    
    def _all_uavs_ready(self) -> bool:
        """Check if all UAVs are ready for mission."""
        if not self.swarm_connector.connectors:
            return False
        
        for uav_id, connector in self.swarm_connector.connectors.items():
            if not connector.is_healthy():
                return False
        
        return True
    
    def _all_uavs_at_altitude(self) -> bool:
        """Check if all UAVs have reached takeoff altitude."""
        target_altitude = self.get_parameter('uav.takeoff_height').value
        tolerance = self.get_parameter('control.position_tolerance').value
        
        for uav_id, connector in self.swarm_connector.connectors.items():
            status = connector.get_status()
            if not status.position or abs(status.position.z - target_altitude) > tolerance:
                return False
        
        return True
    
    def _all_uavs_landed(self) -> bool:
        """Check if all UAVs have landed."""
        for uav_id, connector in self.swarm_connector.connectors.items():
            status = connector.get_status()
            if status.in_air:
                return False
        
        return True
    
    def _coverage_complete(self) -> bool:
        """Check if coverage mission is complete."""
        # This will be implemented with the coverage node
        # For now, return False to keep mission running
        return False
    
    def _formation_stable(self) -> bool:
        """Check if formation is stable."""
        # Check if all UAVs are close to their formation positions
        tolerance = self.get_parameter('control.position_tolerance').value
        
        for uav_id, goal in self.uav_goals.items():
            if uav_id in self.swarm_connector.connectors:
                status = self.swarm_connector.connectors[uav_id].get_status()
                if status.position:
                    distance = TransformUtils.calculate_distance(status.position, goal.position)
                    if distance > tolerance:
                        return False
        
        return True
    
    def _check_safety_conditions(self):
        """Check safety conditions and trigger emergency if needed."""
        # Check minimum separation between UAVs
        min_separation = self.get_parameter('safety.min_separation').value
        
        uav_positions = []
        for uav_id, connector in self.swarm_connector.connectors.items():
            status = connector.get_status()
            if status.position:
                uav_positions.append((uav_id, status.position))
        
        # Check pairwise distances
        for i, (uav1_id, pos1) in enumerate(uav_positions):
            for j, (uav2_id, pos2) in enumerate(uav_positions[i+1:], i+1):
                distance = TransformUtils.calculate_distance(pos1, pos2)
                if distance < min_separation:
                    self.get_logger().error(
                        f"Safety violation: {uav1_id} and {uav2_id} "
                        f"too close ({distance:.2f}m < {min_separation}m)"
                    )
                    self._handle_emergency()
                    return
    
    def _handle_emergency(self):
        """Handle emergency situations."""
        self.mission_state = MissionState.EMERGENCY
        
        # Publish emergency message
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_pub.publish(emergency_msg)
        
        self.get_logger().error("EMERGENCY: Stopping all UAVs")
    
    def _coverage_map_callback(self, msg: OccupancyGrid):
        """Handle coverage map updates."""
        # Process coverage map and update mission state
        # This will be implemented with the coverage node
        pass
    
    def _publish_status_updates(self):
        """Publish status updates."""
        # Publish swarm status
        status_msg = String()
        status_msg.data = f"State: {self.mission_state.value}, "
        status_msg.data += f"UAVs: {len(self.swarm_connector.connectors)}"
        self.swarm_status_pub.publish(status_msg)
        
        # Publish mission status
        mission_msg = String()
        mission_msg.data = f"Mission: {self.mission_state.value}, "
        if self.mission_start_time > 0:
            elapsed = time.time() - self.mission_start_time
            mission_msg.data += f"Time: {elapsed:.1f}s"
        self.mission_status_pub.publish(mission_msg)
        
        # Publish UAV goals visualization
        self._publish_goals_visualization()
    
    def _publish_goals_visualization(self):
        """Publish UAV goals for visualization in RViz."""
        marker_array = MarkerArray()
        
        for uav_id, goal in self.uav_goals.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = f"goal_{uav_id}"
            marker.id = hash(uav_id) % 1000
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position = goal.position
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 0.5
            marker.scale.y = 0.5
            marker.scale.z = 0.5
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.uav_goals_pub.publish(marker_array)
    
    def start_mission(self):
        """Start the swarm mission."""
        if self.mission_state != MissionState.IDLE:
            self.get_logger().warn("Mission already in progress")
            return False
        
        self.mission_state = MissionState.INITIALIZING
        self.mission_start_time = time.time()
        self.get_logger().info("Starting swarm mission")
        return True
    
    def stop_mission(self):
        """Stop the swarm mission."""
        if self.mission_state == MissionState.IDLE:
            self.get_logger().warn("No mission in progress")
            return False
        
        self.mission_state = MissionState.LANDING
        self.get_logger().info("Stopping swarm mission")
        return True
    
    def cleanup(self):
        """Clean up resources before shutdown."""
        # Stop mission
        if self.mission_state != MissionState.IDLE:
            self.stop_mission()
        
        # Disconnect from UAVs
        if self.mavsdk_loop and self.mavsdk_loop.is_running():
            asyncio.run_coroutine_threadsafe(
                self.swarm_connector.disconnect_all(),
                self.mavsdk_loop
            )
        
        # Stop MAVSDK event loop
        if self.mavsdk_loop and self.mavsdk_loop.is_running():
            self.mavsdk_loop.stop()
        
        # Wait for thread to finish
        if self.mavsdk_thread and self.mavsdk_thread.is_alive():
            self.mavsdk_thread.join(timeout=5.0)


def main(args=None):
    """Main function for the swarm coordinator node."""
    rclpy.init(args=args)
    
    try:
        coordinator = SwarmCoordinator()
        rclpy.spin(coordinator)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in swarm coordinator: {e}")
    finally:
        if 'coordinator' in locals():
            coordinator.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
