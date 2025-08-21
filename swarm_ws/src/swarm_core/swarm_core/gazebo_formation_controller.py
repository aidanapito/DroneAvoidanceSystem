#!/usr/bin/env python3
"""
Gazebo Formation Flying Controller

This node controls drone formation flying in Gazebo simulation.
Integrates with ROS 2 for communication and Gazebo for physics simulation.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import time
import math
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum

# ROS 2 messages
from geometry_msgs.msg import Point, Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool, Float32
from visualization_msgs.msg import MarkerArray, Marker

# Custom messages
from std_srvs.srv import Trigger
from geometry_msgs.msg import TwistStamped


class FormationType(Enum):
    """Formation types available."""
    TRIANGLE = "triangle"
    LINE = "line"
    SQUARE = "square"
    V_FORMATION = "v_formation"
    CIRCLE = "circle"
    DIAMOND = "diamond"


@dataclass
class FormationConfig:
    """Configuration for a formation pattern."""
    name: str
    positions: List[Tuple[float, float, float]]
    spacing: float
    altitude: float


class GazeboFormationController(Node):
    """
    Formation flying controller for Gazebo simulation.
    
    This node:
    - Manages different formation patterns
    - Controls drone positions in formation
    - Handles formation transitions
    - Publishes formation status and visualization
    """
    
    def __init__(self):
        """Initialize the formation controller node."""
        super().__init__('gazebo_formation_controller')
        
        # Initialize parameters
        self._init_parameters()
        
        # Initialize formation patterns
        self._init_formation_patterns()
        
        # Current formation state
        self.current_formation = FormationType.TRIANGLE
        self.formation_center = np.array([0.0, 0.0, 5.0])
        self.formation_spacing = 3.0
        self.formation_altitude = 5.0
        
        # Drone tracking
        self.drone_poses: Dict[str, Pose] = {}
        self.drone_targets: Dict[str, Point] = {}
        self.formation_stable = False
        
        # Control parameters
        self.position_tolerance = 0.5
        self.velocity_tolerance = 0.1
        self.max_velocity = 2.0
        
        # Initialize ROS 2 components
        self._init_ros_components()
        
        # Start control loop
        self.control_timer = self.create_timer(
            1.0 / self.get_parameter('control.rate').value,
            self._control_loop_callback
        )
        
        # Formation change timer
        self.formation_change_timer = self.create_timer(
            self.get_parameter('formation.change_interval').value,
            self._change_formation_callback
        )
        
        self.get_logger().info("Gazebo Formation Controller initialized")
    
    def _init_parameters(self):
        """Initialize node parameters."""
        # Control parameters
        self.declare_parameter('control.rate', 20.0)
        self.declare_parameter('control.position_tolerance', 0.5)
        self.declare_parameter('control.velocity_tolerance', 0.1)
        self.declare_parameter('control.max_velocity', 2.0)
        
        # Formation parameters
        self.declare_parameter('formation.spacing', 3.0)
        self.declare_parameter('formation.altitude', 5.0)
        self.declare_parameter('formation.change_interval', 15.0)
        self.declare_parameter('formation.enable_auto_change', True)
        
        # Drone parameters
        self.declare_parameter('drones.count', 3)
        self.declare_parameter('drones.names', ['drone1', 'drone2', 'drone3'])
        
        # Load parameters
        self.formation_spacing = self.get_parameter('formation.spacing').value
        self.formation_altitude = self.get_parameter('formation.altitude').value
        self.position_tolerance = self.get_parameter('control.position_tolerance').value
        self.max_velocity = self.get_parameter('control.max_velocity').value
    
    def _init_formation_patterns(self):
        """Initialize available formation patterns."""
        self.formation_patterns = {
            FormationType.TRIANGLE: FormationConfig(
                name="Triangle",
                positions=[
                    (0.0, 0.0, 0.0),           # Center
                    (-1.0, -1.0, 0.0),         # Left back
                    (1.0, -1.0, 0.0),          # Right back
                ],
                spacing=self.formation_spacing,
                altitude=self.formation_altitude
            ),
            FormationType.LINE: FormationConfig(
                name="Line",
                positions=[
                    (-1.0, 0.0, 0.0),          # Left
                    (0.0, 0.0, 0.0),           # Center
                    (1.0, 0.0, 0.0),           # Right
                ],
                spacing=self.formation_spacing,
                altitude=self.formation_altitude
            ),
            FormationType.SQUARE: FormationConfig(
                name="Square",
                positions=[
                    (-0.5, -0.5, 0.0),         # Bottom left
                    (0.5, -0.5, 0.0),          # Bottom right
                    (0.0, 0.5, 0.0),           # Top center
                ],
                spacing=self.formation_spacing,
                altitude=self.formation_altitude
            ),
            FormationType.V_FORMATION: FormationConfig(
                name="V Formation",
                positions=[
                    (0.0, 0.0, 0.0),           # Leader
                    (-1.0, -1.0, 0.0),         # Left wing
                    (1.0, -1.0, 0.0),          # Right wing
                ],
                spacing=self.formation_spacing,
                altitude=self.formation_altitude
            ),
            FormationType.CIRCLE: FormationConfig(
                name="Circle",
                positions=[
                    (0.0, 1.0, 0.0),           # North
                    (-0.866, -0.5, 0.0),       # Southwest
                    (0.866, -0.5, 0.0),        # Southeast
                ],
                spacing=self.formation_spacing,
                altitude=self.formation_altitude
            ),
            FormationType.DIAMOND: FormationConfig(
                name="Diamond",
                positions=[
                    (0.0, 1.0, 0.0),           # North
                    (-1.0, 0.0, 0.0),          # West
                    (0.0, -1.0, 0.0),          # South
                    (1.0, 0.0, 0.0),           # East
                ],
                spacing=self.formation_spacing,
                altitude=self.formation_altitude
            )
        }
    
    def _init_ros_components(self):
        """Initialize ROS 2 publishers, subscribers, and services."""
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        fast_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Publishers
        self.formation_status_pub = self.create_publisher(
            String, '/formation/status', reliable_qos
        )
        
        self.formation_visualization_pub = self.create_publisher(
            MarkerArray, '/formation/visualization', fast_qos
        )
        
        # Drone control publishers
        self.drone_control_pubs = {}
        drone_names = self.get_parameter('drones.names').value
        for drone_name in drone_names:
            pub = self.create_publisher(
                TwistStamped, f'/{drone_name}/cmd_vel', fast_qos
            )
            self.drone_control_pubs[drone_name] = pub
        
        # Subscribers
        self.drone_pose_subs = {}
        for drone_name in drone_names:
            sub = self.create_subscription(
                Odometry, f'/{drone_name}/odom',
                lambda msg, name=drone_name: self._drone_pose_callback(msg, name),
                fast_qos
            )
            self.drone_pose_subs[drone_name] = sub
        
        # Services
        self.change_formation_srv = self.create_service(
            Trigger, '/formation/change',
            self._change_formation_service_callback
        )
        
        self.set_formation_srv = self.create_service(
            Trigger, '/formation/set',
            self._set_formation_service_callback
        )
        
        # Callback groups
        self.callback_group = ReentrantCallbackGroup()
    
    def _drone_pose_callback(self, msg: Odometry, drone_name: str):
        """Handle drone pose updates."""
        pose = Pose()
        pose.position = msg.pose.pose.position
        pose.orientation = msg.pose.pose.orientation
        
        self.drone_poses[drone_name] = pose
        
        # Update target if not set
        if drone_name not in self.drone_targets:
            self._update_formation_targets()
    
    def _control_loop_callback(self):
        """Main control loop for formation flying."""
        try:
            # Update formation targets
            self._update_formation_targets()
            
            # Control each drone
            self._control_drones()
            
            # Check formation stability
            self._check_formation_stability()
            
            # Publish status and visualization
            self._publish_status()
            self._publish_visualization()
            
        except Exception as e:
            self.get_logger().error(f"Error in control loop: {e}")
    
    def _change_formation_callback(self):
        """Timer callback for automatic formation changes."""
        if self.get_parameter('formation.enable_auto_change').value:
            self._change_to_next_formation()
    
    def _update_formation_targets(self):
        """Update target positions for all drones based on current formation."""
        if not self.formation_patterns:
            return
        
        config = self.formation_patterns[self.current_formation]
        drone_names = list(self.drone_control_pubs.keys())
        
        for i, drone_name in enumerate(drone_names):
            if i < len(config.positions):
                # Scale positions by spacing and add center offset
                x = config.positions[i][0] * self.formation_spacing + self.formation_center[0]
                y = config.positions[i][1] * self.formation_spacing + self.formation_center[1]
                z = config.positions[i][2] + self.formation_altitude
                
                target = Point(x=x, y=y, z=z)
                self.drone_targets[drone_name] = target
    
    def _control_drones(self):
        """Control each drone to reach its formation target."""
        for drone_name, target in self.drone_targets.items():
            if drone_name in self.drone_poses and drone_name in self.drone_control_pubs:
                current_pose = self.drone_poses[drone_name]
                
                # Calculate control command
                cmd = self._calculate_control_command(current_pose, target)
                
                # Publish control command
                if cmd:
                    self.drone_control_pubs[drone_name].publish(cmd)
    
    def _calculate_control_command(self, current_pose: Pose, target: Point) -> Optional[TwistStamped]:
        """Calculate control command for a drone."""
        # Calculate position error
        pos_error = np.array([
            target.x - current_pose.position.x,
            target.y - current_pose.position.y,
            target.z - current_pose.position.z
        ])
        
        # Calculate distance to target
        distance = np.linalg.norm(pos_error)
        
        if distance < self.position_tolerance:
            # Close enough to target, stop
            return None
        
        # Calculate desired velocity (proportional control)
        desired_velocity = pos_error / distance * self.max_velocity
        
        # Limit velocity
        if np.linalg.norm(desired_velocity) > self.max_velocity:
            desired_velocity = desired_velocity / np.linalg.norm(desired_velocity) * self.max_velocity
        
        # Create twist command
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = "map"
        
        cmd.twist.linear.x = desired_velocity[0]
        cmd.twist.linear.y = desired_velocity[1]
        cmd.twist.linear.z = desired_velocity[2]
        
        return cmd
    
    def _check_formation_stability(self):
        """Check if the formation is stable (all drones near targets)."""
        if not self.drone_targets:
            self.formation_stable = False
            return
        
        all_stable = True
        for drone_name, target in self.drone_targets.items():
            if drone_name in self.drone_poses:
                current_pose = self.drone_poses[drone_name]
                distance = math.sqrt(
                    (target.x - current_pose.position.x) ** 2 +
                    (target.y - current_pose.position.y) ** 2 +
                    (target.z - current_pose.position.z) ** 2
                )
                
                if distance > self.position_tolerance:
                    all_stable = False
                    break
        
        self.formation_stable = all_stable
    
    def _change_to_next_formation(self):
        """Change to the next formation pattern."""
        formations = list(self.formation_patterns.keys())
        current_index = formations.index(self.current_formation)
        next_index = (current_index + 1) % len(formations)
        
        self.current_formation = formations[next_index]
        self.get_logger().info(f"Changed to {self.current_formation.value} formation")
        
        # Update targets
        self._update_formation_targets()
    
    def _change_formation_service_callback(self, request, response):
        """Service callback to change formation."""
        self._change_to_next_formation()
        response.success = True
        response.message = f"Changed to {self.current_formation.value} formation"
        return response
    
    def _set_formation_service_callback(self, request, response):
        """Service callback to set specific formation."""
        # This could be extended to accept formation type as parameter
        self._change_to_next_formation()
        response.success = True
        response.message = f"Set to {self.current_formation.value} formation"
        return response
    
    def _publish_status(self):
        """Publish formation status."""
        status_msg = String()
        status_msg.data = (
            f"Formation: {self.current_formation.value}, "
            f"Stable: {self.formation_stable}, "
            f"Center: ({self.formation_center[0]:.1f}, {self.formation_center[1]:.1f}, {self.formation_center[2]:.1f}), "
            f"Drones: {len(self.drone_poses)}"
        )
        
        self.formation_status_pub.publish(status_msg)
    
    def _publish_visualization(self):
        """Publish formation visualization markers."""
        marker_array = MarkerArray()
        
        # Drone position markers
        for i, (drone_name, pose) in enumerate(self.drone_poses.items()):
            # Current position marker
            drone_marker = Marker()
            drone_marker.header.frame_id = "map"
            drone_marker.header.stamp = self.get_clock().now().to_msg()
            drone_marker.ns = f"drone_{drone_name}"
            drone_marker.id = i * 2
            drone_marker.type = Marker.SPHERE
            drone_marker.action = Marker.ADD
            
            drone_marker.pose.position = pose.position
            drone_marker.pose.orientation = pose.orientation
            
            drone_marker.scale.x = 0.3
            drone_marker.scale.y = 0.3
            drone_marker.scale.z = 0.3
            
            # Color based on formation stability
            if self.formation_stable:
                drone_marker.color.r = 0.0
                drone_marker.color.g = 1.0
                drone_marker.color.b = 0.0
            else:
                drone_marker.color.r = 1.0
                drone_marker.color.g = 0.0
                drone_marker.color.b = 0.0
            
            drone_marker.color.a = 0.8
            marker_array.markers.append(drone_marker)
            
            # Target position marker
            if drone_name in self.drone_targets:
                target_marker = Marker()
                target_marker.header.frame_id = "map"
                target_marker.header.stamp = self.get_clock().now().to_msg()
                target_marker.ns = f"target_{drone_name}"
                target_marker.id = i * 2 + 1
                target_marker.type = Marker.SPHERE
                target_marker.action = Marker.ADD
                
                target = self.drone_targets[drone_name]
                target_marker.pose.position = target
                target_marker.pose.orientation.w = 1.0
                
                target_marker.scale.x = 0.15
                target_marker.scale.y = 0.15
                target_marker.scale.z = 0.15
                
                target_marker.color.r = 0.0
                target_marker.color.g = 0.0
                target_marker.color.b = 1.0
                target_marker.color.a = 0.6
                
                marker_array.markers.append(target_marker)
        
        # Formation center marker
        center_marker = Marker()
        center_marker.header.frame_id = "map"
        center_marker.header.stamp = self.get_clock().now().to_msg()
        center_marker.ns = "formation_center"
        center_marker.id = 1000
        center_marker.type = Marker.CUBE
        center_marker.action = Marker.ADD
        
        center_marker.pose.position.x = self.formation_center[0]
        center_marker.pose.position.y = self.formation_center[1]
        center_marker.pose.position.z = self.formation_center[2]
        center_marker.pose.orientation.w = 1.0
        
        center_marker.scale.x = 0.2
        center_marker.scale.y = 0.2
        center_marker.scale.z = 0.2
        
        center_marker.color.r = 1.0
        center_marker.color.g = 1.0
        center_marker.color.b = 0.0
        center_marker.color.a = 0.8
        
        marker_array.markers.append(center_marker)
        
        self.formation_visualization_pub.publish(marker_array)
    
    def move_formation(self, new_center: Tuple[float, float, float]):
        """Move the entire formation to a new center."""
        self.formation_center = np.array(new_center)
        self._update_formation_targets()
        self.get_logger().info(f"Moved formation to {new_center}")
    
    def cleanup(self):
        """Clean up resources before shutdown."""
        # Stop all drones
        for drone_name, pub in self.drone_control_pubs.items():
            stop_cmd = TwistStamped()
            stop_cmd.header.stamp = self.get_clock().now().to_msg()
            stop_cmd.header.frame_id = "map"
            pub.publish(stop_cmd)


def main(args=None):
    """Main function for the formation controller node."""
    rclpy.init(args=args)
    
    try:
        controller = GazeboFormationController()
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in formation controller: {e}")
    finally:
        if 'controller' in locals():
            controller.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
