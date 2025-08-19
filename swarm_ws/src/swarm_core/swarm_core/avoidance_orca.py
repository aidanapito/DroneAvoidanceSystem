#!/usr/bin/env python3
"""
ORCA Collision Avoidance Node

Implements the ORCA (Optimal Reciprocal Collision Avoidance) algorithm
for safe velocity planning in multi-UAV swarms.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup

import numpy as np
import math
from typing import Dict, List, Optional, Tuple, Set
from dataclasses import dataclass
from enum import Enum

# ROS 2 messages
from geometry_msgs.msg import Point, Twist, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool
from visualization_msgs.msg import MarkerArray, Marker

# Local imports
from .utils.transforms import TransformUtils, SafetyUtils


class ORCAState(Enum):
    """Enumeration of ORCA algorithm states."""
    IDLE = "idle"
    ACTIVE = "active"
    EMERGENCY = "emergency"


@dataclass
class UAVState:
    """Data class for UAV state information."""
    uav_id: str
    position: Point
    velocity: Point
    radius: float
    max_speed: float
    timestamp: float


@dataclass
class ORCAConstraint:
    """Data class for ORCA velocity constraints."""
    uav_id: str
    constraint_line: Tuple[Point, Point]  # Two points defining the constraint line
    constraint_side: bool  # True if velocity should be on left side of line
    timestamp: float


class ORCACollisionAvoidance(Node):
    """
    ORCA collision avoidance node.
    
    Responsibilities:
    - Monitor positions and velocities of all UAVs
    - Compute ORCA velocity constraints
    - Generate safe velocity setpoints
    - Publish collision avoidance information
    """
    
    def __init__(self):
        """Initialize the ORCA collision avoidance node."""
        super().__init__('avoidance_orca')
        
        # Initialize parameters
        self._init_parameters()
        
        # Initialize state
        self.orca_state = ORCAState.IDLE
        self.uav_states: Dict[str, UAVState] = {}
        self.orca_constraints: Dict[str, List[ORCAConstraint]] = {}
        
        # Performance tracking
        self.collision_avoidance_count = 0
        self.min_separation_history: List[float] = []
        
        # Initialize ROS 2 components
        self._init_ros_components()
        
        # Start main control loop
        self.control_timer = self.create_timer(
            1.0 / self.get_parameter('orca.update_rate').value,
            self._control_loop_callback
        )
        
        self.get_logger().info("ORCA Collision Avoidance initialized")
    
    def _init_parameters(self):
        """Initialize node parameters from config file."""
        # ORCA parameters
        self.declare_parameter('orca.neighbor_dist', 10.0)
        self.declare_parameter('orca.time_horizon', 2.0)
        self.declare_parameter('orca.max_speed', 3.0)
        self.declare_parameter('orca.safety_margin', 0.2)
        self.declare_parameter('orca.update_rate', 50.0)
        
        # UAV parameters
        self.declare_parameter('uav.safety_radius', 1.5)
        self.declare_parameter('uav.max_velocity', 3.0)
        
        # Safety parameters
        self.declare_parameter('safety.min_separation', 1.0)
        self.declare_parameter('safety.emergency_stop_distance', 0.8)
    
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
        self.safe_velocities_pub = self.create_publisher(
            Twist, '/swarm/safe_velocities', fast_qos
        )
        
        self.collision_risk_pub = self.create_publisher(
            Bool, '/swarm/collision_risk', fast_qos
        )
        
        self.min_separation_pub = self.create_publisher(
            Float32, '/swarm/min_separation', fast_qos
        )
        
        self.orca_constraints_pub = self.create_publisher(
            MarkerArray, '/swarm/orca_constraints', fast_qos
        )
        
        # Subscribers
        self.odom_subscribers = {}
        self._setup_odometry_subscribers()
        
        # Callback groups for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
    
    def _setup_odometry_subscribers(self):
        """Set up odometry subscribers for all UAVs."""
        # This would typically be configured based on the number of UAVs
        # For now, we'll set up subscribers for 3 UAVs
        uav_names = ["uav1", "uav2", "uav3"]
        
        for uav_name in uav_names:
            topic_name = f"/{uav_name}/odometry"
            subscriber = self.create_subscription(
                Odometry, topic_name,
                lambda msg, name=uav_name: self._odometry_callback(msg, name),
                QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT)
            )
            self.odom_subscribers[uav_name] = subscriber
    
    def _odometry_callback(self, msg: Odometry, uav_id: str):
        """Handle odometry updates from UAVs."""
        try:
            # Extract position and velocity
            position = msg.pose.pose.position
            velocity = msg.twist.twist.linear
            
            # Update UAV state
            uav_state = UAVState(
                uav_id=uav_id,
                position=position,
                velocity=velocity,
                radius=self.get_parameter('uav.safety_radius').value,
                max_speed=self.get_parameter('uav.max_velocity').value,
                timestamp=self.get_clock().now().nanoseconds / 1e9
            )
            
            self.uav_states[uav_id] = uav_state
            
        except Exception as e:
            self.get_logger().error(f"Error processing odometry from {uav_id}: {e}")
    
    def _control_loop_callback(self):
        """Main control loop callback."""
        try:
            # Check if we have enough UAVs for collision avoidance
            if len(self.uav_states) < 2:
                return
            
            # Update ORCA state
            self._update_orca_state()
            
            # Compute ORCA constraints
            self._compute_orca_constraints()
            
            # Generate safe velocities
            safe_velocities = self._generate_safe_velocities()
            
            # Publish results
            self._publish_results(safe_velocities)
            
            # Update performance metrics
            self._update_performance_metrics()
            
        except Exception as e:
            self.get_logger().error(f"Error in ORCA control loop: {e}")
            self._handle_emergency()
    
    def _update_orca_state(self):
        """Update ORCA algorithm state."""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Check for emergency conditions
        min_separation = self._calculate_min_separation()
        emergency_distance = self.get_parameter('safety.emergency_stop_distance').value
        
        if min_separation < emergency_distance:
            self.orca_state = ORCAState.EMERGENCY
            self.get_logger().error(f"EMERGENCY: Min separation {min_separation:.2f}m < {emergency_distance}m")
        elif min_separation < self.get_parameter('safety.min_separation').value:
            self.orca_state = ORCAState.ACTIVE
        else:
            self.orca_state = ORCAState.IDLE
    
    def _compute_orca_constraints(self):
        """Compute ORCA velocity constraints for all UAV pairs."""
        self.orca_constraints.clear()
        
        uav_ids = list(self.uav_states.keys())
        
        for i, uav1_id in enumerate(uav_ids):
            for j, uav2_id in enumerate(uav_ids[i+1:], i+1):
                self._compute_pairwise_orca_constraints(uav1_id, uav2_id)
    
    def _compute_pairwise_orca_constraints(self, uav1_id: str, uav2_id: str):
        """Compute ORCA constraints between two UAVs."""
        uav1 = self.uav_states.get(uav1_id)
        uav2 = self.uav_states.get(uav2_id)
        
        if not uav1 or not uav2:
            return
        
        # Calculate relative position and velocity
        rel_pos = Point()
        rel_pos.x = uav2.position.x - uav1.position.x
        rel_pos.y = uav2.position.y - uav1.position.y
        rel_pos.z = uav2.position.z - uav1.position.z
        
        rel_vel = Point()
        rel_vel.x = uav2.velocity.x - uav1.velocity.x
        rel_vel.y = uav2.velocity.y - uav1.velocity.y
        rel_vel.z = uav2.velocity.z - uav1.velocity.z
        
        # Calculate distance
        distance = TransformUtils.calculate_distance(rel_pos, Point())
        
        # Check if UAVs are within neighbor distance
        neighbor_dist = self.get_parameter('orca.neighbor_dist').value
        if distance > neighbor_dist:
            return
        
        # Calculate combined radius
        combined_radius = uav1.radius + uav2.radius + self.get_parameter('orca.safety_margin').value
        
        # Check if collision is imminent
        if distance < combined_radius:
            # Immediate collision - emergency stop
            self._add_emergency_constraint(uav1_id, uav2_id)
            return
        
        # Compute ORCA constraints
        time_horizon = self.get_parameter('orca.time_horizon').value
        
        # Calculate closest approach time
        rel_vel_mag = TransformUtils.calculate_distance(rel_vel, Point())
        
        if rel_vel_mag < 1e-6:  # Nearly stationary relative to each other
            # UAVs are not moving relative to each other
            # Create constraint to maintain separation
            self._add_separation_constraint(uav1_id, uav2_id, rel_pos, combined_radius)
            return
        
        # Calculate time to closest approach
        t_ca = -np.dot([rel_pos.x, rel_pos.y, rel_pos.z], 
                       [rel_vel.x, rel_vel.y, rel_vel.z]) / (rel_vel_mag * rel_vel_mag)
        
        # Clamp to time horizon
        t_ca = max(0.0, min(t_ca, time_horizon))
        
        # Calculate closest approach distance
        d_ca = math.sqrt(distance * distance - (t_ca * rel_vel_mag) * (t_ca * rel_vel_mag))
        
        # Check if closest approach is too close
        if d_ca < combined_radius:
            # Calculate ORCA constraint
            self._add_orca_constraint(uav1_id, uav2_id, rel_pos, rel_vel, 
                                    combined_radius, t_ca, d_ca)
    
    def _add_emergency_constraint(self, uav1_id: str, uav2_id: str):
        """Add emergency constraint to stop both UAVs."""
        # Create emergency stop constraints
        emergency_constraint1 = ORCAConstraint(
            uav_id=uav2_id,
            constraint_line=(Point(x=0, y=0, z=0), Point(x=1, y=0, z=0)),
            constraint_side=True,
            timestamp=self.get_clock().now().nanoseconds / 1e9
        )
        
        emergency_constraint2 = ORCAConstraint(
            uav_id=uav1_id,
            constraint_line=(Point(x=0, y=0, z=0), Point(x=1, y=0, z=0)),
            constraint_side=True,
            timestamp=self.get_clock().now().nanoseconds / 1e9
        )
        
        if uav1_id not in self.orca_constraints:
            self.orca_constraints[uav1_id] = []
        if uav2_id not in self.orca_constraints:
            self.orca_constraints[uav2_id] = []
        
        self.orca_constraints[uav1_id].append(emergency_constraint1)
        self.orca_constraints[uav2_id].append(emergency_constraint2)
    
    def _add_separation_constraint(self, uav1_id: str, uav2_id: str, 
                                 rel_pos: Point, combined_radius: float):
        """Add constraint to maintain separation between stationary UAVs."""
        # Create constraint perpendicular to separation vector
        constraint_dir = Point()
        constraint_dir.x = -rel_pos.y
        constraint_dir.y = rel_pos.x
        constraint_dir.z = 0.0
        
        # Normalize
        mag = TransformUtils.calculate_distance(constraint_dir, Point())
        if mag > 1e-6:
            constraint_dir.x /= mag
            constraint_dir.y /= mag
        
        # Create constraint line
        constraint_point = Point()
        constraint_point.x = rel_pos.x + constraint_dir.x * combined_radius
        constraint_point.y = rel_pos.y + constraint_dir.y * combined_radius
        constraint_point.z = rel_pos.z
        
        constraint = ORCAConstraint(
            uav_id=uav2_id,
            constraint_line=(constraint_point, constraint_dir),
            constraint_side=True,
            timestamp=self.get_clock().now().nanoseconds / 1e9
        )
        
        if uav1_id not in self.orca_constraints:
            self.orca_constraints[uav1_id] = []
        self.orca_constraints[uav1_id].append(constraint)
    
    def _add_orca_constraint(self, uav1_id: str, uav2_id: str, rel_pos: Point, 
                            rel_vel: Point, combined_radius: float, 
                            t_ca: float, d_ca: float):
        """Add ORCA velocity constraint between two UAVs."""
        # Calculate ORCA constraint parameters
        # This is a simplified version - full ORCA implementation would be more complex
        
        # Calculate constraint direction (perpendicular to relative velocity)
        constraint_dir = Point()
        constraint_dir.x = -rel_vel.y
        constraint_dir.y = rel_vel.x
        constraint_dir.z = 0.0
        
        # Normalize
        mag = TransformUtils.calculate_distance(constraint_dir, Point())
        if mag > 1e-6:
            constraint_dir.x /= mag
            constraint_dir.y /= mag
        
        # Calculate constraint point
        constraint_point = Point()
        constraint_point.x = rel_pos.x + constraint_dir.x * combined_radius
        constraint_point.y = rel_pos.y + constraint_dir.y * combined_radius
        constraint_point.z = rel_pos.z
        
        # Create constraint
        constraint = ORCAConstraint(
            uav_id=uav2_id,
            constraint_line=(constraint_point, constraint_dir),
            constraint_side=True,
            timestamp=self.get_clock().now().nanoseconds / 1e9
        )
        
        if uav1_id not in self.orca_constraints:
            self.orca_constraints[uav1_id] = []
        self.orca_constraints[uav1_id].append(constraint)
    
    def _generate_safe_velocities(self) -> Dict[str, Twist]:
        """Generate safe velocity setpoints for all UAVs."""
        safe_velocities = {}
        
        for uav_id, uav_state in self.uav_states.items():
            # Start with desired velocity (could come from coverage planner)
            desired_velocity = Twist()
            desired_velocity.linear = uav_state.velocity
            
            # Apply ORCA constraints
            safe_velocity = self._apply_orca_constraints(uav_id, desired_velocity)
            
            # Apply safety limits
            safe_velocity = self._apply_safety_limits(uav_id, safe_velocity)
            
            safe_velocities[uav_id] = safe_velocity
        
        return safe_velocities
    
    def _apply_orca_constraints(self, uav_id: str, desired_velocity: Twist) -> Twist:
        """Apply ORCA constraints to desired velocity."""
        if uav_id not in self.orca_constraints:
            return desired_velocity
        
        # For now, implement a simplified constraint application
        # Full ORCA would solve a linear programming problem
        
        safe_velocity = Twist()
        safe_velocity.linear.x = desired_velocity.linear.x
        safe_velocity.linear.y = desired_velocity.linear.y
        safe_velocity.linear.z = desired_velocity.linear.z
        
        # Apply emergency constraints
        for constraint in self.orca_constraints[uav_id]:
            if constraint.constraint_line[0].x == 0 and constraint.constraint_line[0].y == 0:
                # Emergency constraint - stop the UAV
                safe_velocity.linear.x = 0.0
                safe_velocity.linear.y = 0.0
                safe_velocity.linear.z = 0.0
                break
        
        return safe_velocity
    
    def _apply_safety_limits(self, uav_id: str, velocity: Twist) -> Twist:
        """Apply safety limits to velocity setpoint."""
        uav_state = self.uav_states.get(uav_id)
        if not uav_state:
            return velocity
        
        limited_velocity = Twist()
        
        # Apply maximum speed limit
        max_speed = min(uav_state.max_speed, self.get_parameter('orca.max_speed').value)
        vel_mag = TransformUtils.calculate_distance(velocity.linear, Point())
        
        if vel_mag > max_speed:
            scale = max_speed / vel_mag
            limited_velocity.linear.x = velocity.linear.x * scale
            limited_velocity.linear.y = velocity.linear.y * scale
            limited_velocity.linear.z = velocity.linear.z * scale
        else:
            limited_velocity.linear.x = velocity.linear.x
            limited_velocity.linear.y = velocity.linear.y
            limited_velocity.linear.z = velocity.linear.z
        
        # Apply angular velocity limits
        limited_velocity.angular.x = velocity.angular.x
        limited_velocity.angular.y = velocity.angular.y
        limited_velocity.angular.z = max(-1.0, min(1.0, velocity.angular.z))
        
        return limited_velocity
    
    def _calculate_min_separation(self) -> float:
        """Calculate minimum separation between any two UAVs."""
        if len(self.uav_states) < 2:
            return float('inf')
        
        min_separation = float('inf')
        uav_positions = list(self.uav_states.values())
        
        for i, uav1 in enumerate(uav_positions):
            for j, uav2 in enumerate(uav_positions[i+1:], i+1):
                separation = TransformUtils.calculate_distance(uav1.position, uav2.position)
                min_separation = min(min_separation, separation)
        
        return min_separation
    
    def _publish_results(self, safe_velocities: Dict[str, Twist]):
        """Publish collision avoidance results."""
        # Publish safe velocities for each UAV
        for uav_id, velocity in safe_velocities.items():
            # This would typically be published to individual UAV topics
            # For now, publish to a general topic
            self.safe_velocities_pub.publish(velocity)
        
        # Publish collision risk status
        collision_risk_msg = Bool()
        collision_risk_msg.data = self.orca_state == ORCAState.EMERGENCY
        self.collision_risk_pub.publish(collision_risk_msg)
        
        # Publish minimum separation
        min_separation_msg = Float32()
        min_separation_msg.data = self._calculate_min_separation()
        self.min_separation_pub.publish(min_separation_msg)
        
        # Publish ORCA constraints visualization
        self._publish_constraints_visualization()
    
    def _publish_constraints_visualization(self):
        """Publish ORCA constraints for visualization in RViz."""
        marker_array = MarkerArray()
        
        for uav_id, constraints in self.orca_constraints.items():
            for i, constraint in enumerate(constraints):
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = f"orca_constraint_{uav_id}_{i}"
                marker.id = hash(f"{uav_id}_{i}") % 1000
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                
                # Create constraint line visualization
                # This is a simplified visualization
                marker.points = [constraint.constraint_line[0], constraint.constraint_line[1]]
                
                marker.scale.x = 0.1  # Line width
                
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 0.8
                
                marker_array.markers.append(marker)
        
        self.orca_constraints_pub.publish(marker_array)
    
    def _update_performance_metrics(self):
        """Update performance tracking metrics."""
        min_separation = self._calculate_min_separation()
        self.min_separation_history.append(min_separation)
        
        # Keep only recent history
        if len(self.min_separation_history) > 1000:
            self.min_separation_history = self.min_separation_history[-1000:]
        
        # Update collision avoidance count
        if self.orca_state == ORCAState.ACTIVE:
            self.collision_avoidance_count += 1
    
    def _handle_emergency(self):
        """Handle emergency situations."""
        self.orca_state = ORCAState.EMERGENCY
        self.get_logger().error("ORCA EMERGENCY: Stopping all UAVs")
        
        # Publish emergency stop commands
        emergency_velocity = Twist()
        emergency_velocity.linear.x = 0.0
        emergency_velocity.linear.y = 0.0
        emergency_velocity.linear.z = 0.0
        
        self.safe_velocities_pub.publish(emergency_velocity)
    
    def get_performance_stats(self) -> Dict[str, float]:
        """Get performance statistics."""
        if not self.min_separation_history:
            return {}
        
        return {
            'min_separation_current': self.min_separation_history[-1],
            'min_separation_min': min(self.min_separation_history),
            'min_separation_avg': np.mean(self.min_separation_history),
            'collision_avoidance_count': self.collision_avoidance_count,
            'uav_count': len(self.uav_states)
        }


def main(args=None):
    """Main function for the ORCA collision avoidance node."""
    rclpy.init(args=args)
    
    try:
        avoidance_node = ORCACollisionAvoidance()
        rclpy.spin(avoidance_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error in ORCA collision avoidance: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
