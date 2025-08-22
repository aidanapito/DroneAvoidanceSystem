#!/usr/bin/env python3
"""
ORCA Algorithm Test with Simulated Drone Movements

This script tests the ORCA collision avoidance algorithm by:
- Simulating 3 drones moving in different patterns
- Publishing fake odometry data to test collision avoidance
- Monitoring ORCA responses and safe velocity generation
- Testing various collision scenarios
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.clock import Clock
import math
import time
from typing import Dict, List, Tuple

# ROS 2 messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion, Twist, Pose
from std_msgs.msg import Header
from std_msgs.msg import Bool, Float32

# Local imports
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'src', 'swarm_core', 'swarm_core'))
from utils.transforms import TransformUtils


class DroneSimulator:
    """Simulates individual drone movement patterns."""
    
    def __init__(self, drone_id: str, initial_pos: Tuple[float, float, float]):
        self.drone_id = drone_id
        self.position = list(initial_pos)
        self.velocity = [0.0, 0.0, 0.0]
        self.time_start = time.time()
        
        # Movement parameters
        self.pattern = "circle"  # circle, line, figure8, random
        self.speed = 1.0  # m/s
        self.radius = 3.0  # m for circular patterns
        
    def update_position(self, dt: float):
        """Update drone position based on movement pattern."""
        current_time = time.time() - self.time_start
        
        if self.pattern == "circle":
            # Circular motion
            angle = current_time * 0.5  # Angular velocity
            center_x, center_y, center_z = 0.0, 0.0, 5.0
            
            self.position[0] = center_x + self.radius * math.cos(angle)
            self.position[1] = center_y + self.radius * math.sin(angle)
            self.position[2] = center_z
            
            # Calculate velocity
            self.velocity[0] = -self.speed * math.sin(angle)
            self.velocity[1] = self.speed * math.cos(angle)
            self.velocity[2] = 0.0
            
        elif self.pattern == "line":
            # Back and forth motion
            x_pos = 5.0 * math.sin(current_time * 0.3)
            self.position[0] = x_pos
            self.position[1] = 0.0
            self.position[2] = 5.0
            
            self.velocity[0] = 5.0 * 0.3 * math.cos(current_time * 0.3)
            self.velocity[1] = 0.0
            self.velocity[2] = 0.0
            
        elif self.pattern == "figure8":
            # Figure-8 pattern
            angle = current_time * 0.4
            self.position[0] = 4.0 * math.sin(angle)
            self.position[1] = 3.0 * math.sin(angle) * math.cos(angle)
            self.position[2] = 5.0
            
            # Approximate velocity
            self.velocity[0] = 4.0 * 0.4 * math.cos(angle)
            self.velocity[1] = 3.0 * 0.4 * (math.cos(angle)**2 - math.sin(angle)**2)
            self.velocity[2] = 0.0
            
        elif self.pattern == "random":
            # Random walk with bounds
            import random
            for i in range(3):
                self.velocity[i] += random.uniform(-0.5, 0.5)
                self.velocity[i] = max(-2.0, min(2.0, self.velocity[i]))
                self.position[i] += self.velocity[i] * dt
                
                # Keep within bounds
                if abs(self.position[i]) > 10.0:
                    self.velocity[i] *= -0.5
    
    def get_odometry_msg(self) -> Odometry:
        """Generate odometry message for this drone."""
        msg = Odometry()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = f"{self.drone_id}_base_link"
        
        # Position
        msg.pose.pose.position.x = self.position[0]
        msg.pose.pose.position.y = self.position[1]
        msg.pose.pose.position.z = self.position[2]
        
        # Orientation (simple heading based on velocity)
        if abs(self.velocity[0]) > 0.1 or abs(self.velocity[1]) > 0.1:
            yaw = math.atan2(self.velocity[1], self.velocity[0])
            msg.pose.pose.orientation.w = math.cos(yaw / 2)
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = math.sin(yaw / 2)
        else:
            msg.pose.pose.orientation.w = 1.0
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = 0.0
        
        # Velocity
        msg.twist.twist.linear.x = self.velocity[0]
        msg.twist.twist.linear.y = self.velocity[1]
        msg.twist.twist.linear.z = self.velocity[2]
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        
        return msg


class ORCATestNode(Node):
    """Test node for ORCA collision avoidance algorithm."""
    
    def __init__(self):
        super().__init__('orca_test_node')
        
        # Initialize drone simulators
        self.drones = {
            "uav1": DroneSimulator("uav1", (0.0, 0.0, 5.0)),
            "uav2": DroneSimulator("uav2", (5.0, 0.0, 5.0)),
            "uav3": DroneSimulator("uav3", (0.0, 5.0, 5.0))
        }
        
        # Set different patterns for each drone
        self.drones["uav1"].pattern = "circle"
        self.drones["uav2"].pattern = "line"
        self.drones["uav3"].pattern = "figure8"
        
        # Publishers for each drone's odometry
        self.odom_publishers = {}
        for drone_id in self.drones.keys():
            topic_name = f"/{drone_id}/odometry"
            self.odom_publishers[drone_id] = self.create_publisher(
                Odometry, topic_name, 10
            )
        
        # Subscribers to monitor ORCA output
        self.safe_velocities_sub = self.create_subscription(
            Twist, '/swarm/safe_velocities', self._safe_velocities_callback, 10
        )
        
        self.collision_risk_sub = self.create_subscription(
            Bool, '/swarm/collision_risk', self._collision_risk_callback, 10
        )
        
        self.min_separation_sub = self.create_subscription(
            Float32, '/swarm/min_separation', self._min_separation_callback, 10
        )
        
        # Test state
        self.test_start_time = time.time()
        self.collision_events = 0
        self.avoidance_events = 0
        
        # Start simulation timer
        self.sim_timer = self.create_timer(0.1, self._simulation_timer_callback)  # 10 Hz
        
        # Start test monitoring timer
        self.test_timer = self.create_timer(5.0, self._test_timer_callback)  # 5 Hz
        
        self.get_logger().info("🚁 ORCA Algorithm Test Node initialized")
        self.get_logger().info("   Simulating 3 drones with different movement patterns")
        self.get_logger().info("   Monitoring collision avoidance responses")
    
    def _simulation_timer_callback(self):
        """Update drone positions and publish odometry."""
        current_time = time.time()
        dt = 0.1  # 10 Hz update rate
        
        for drone_id, drone in self.drones.items():
            # Update drone position
            drone.update_position(dt)
            
            # Generate and publish odometry
            odom_msg = drone.get_odometry_msg()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            self.odom_publishers[drone_id].publish(odom_msg)
    
    def _safe_velocities_callback(self, msg: Twist):
        """Handle safe velocities from ORCA."""
        self.avoidance_events += 1
        self.get_logger().info(
            f"🔄 ORCA generated safe velocities: "
            f"vx={msg.linear.x:.2f}, vy={msg.linear.y:.2f}, vz={msg.linear.z:.2f}"
        )
    
    def _collision_risk_callback(self, msg: Bool):
        """Handle collision risk updates."""
        if msg.data:
            self.collision_events += 1
            self.get_logger().warn("🚨 COLLISION RISK DETECTED!")
    
    def _min_separation_callback(self, msg: Float32):
        """Handle minimum separation updates."""
        separation = msg.data
        if separation < 2.0:  # Warning threshold
            self.get_logger().warn(f"⚠️  Low separation: {separation:.2f}m")
    
    def _test_timer_callback(self):
        """Periodic test status update."""
        current_time = time.time()
        elapsed = current_time - self.test_start_time
        
        # Calculate current drone separations
        separations = []
        drone_positions = [(drone_id, drone.position) for drone_id, drone in self.drones.items()]
        
        for i, (id1, pos1) in enumerate(drone_positions):
            for j, (id2, pos2) in enumerate(drone_positions[i+1:], i+1):
                distance = math.sqrt(sum((a - b)**2 for a, b in zip(pos1, pos2)))
                separations.append((id1, id2, distance))
        
        min_separation = min(separations, key=lambda x: x[2]) if separations else (None, None, float('inf'))
        
        self.get_logger().info(
            f"📊 Test Status (t={elapsed:.1f}s): "
            f"Collisions={self.collision_events}, "
            f"Avoidances={self.avoidance_events}, "
            f"Min separation={min_separation[2]:.2f}m "
            f"({min_separation[0]}->{min_separation[1]})"
        )
        
        # Log drone positions
        for drone_id, drone in self.drones.items():
            self.get_logger().info(
                f"   {drone_id}: pos=({drone.position[0]:.1f}, {drone.position[1]:.1f}, {drone.position[2]:.1f}) "
                f"vel=({drone.velocity[0]:.1f}, {drone.velocity[1]:.1f}, {drone.velocity[2]:.1f})"
            )


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        test_node = ORCATestNode()
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted by user")
    except Exception as e:
        test_node.get_logger().error(f"Test error: {e}")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
