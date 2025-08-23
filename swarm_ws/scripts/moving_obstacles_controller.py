#!/usr/bin/env python3
"""
Moving Obstacles Controller for Collision Avoidance Testing

This script controls moving obstacles in the Gazebo world to test
the drone swarm's collision avoidance capabilities.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math
import time

class MovingObstaclesController(Node):
    """Controls moving obstacles for collision avoidance testing."""
    
    def __init__(self):
        super().__init__('moving_obstacles_controller')
        
        # Publishers for each obstacle
        self.obstacle1_pub = self.create_publisher(Twist, '/obstacle1/cmd_vel', 10)
        self.obstacle2_pub = self.create_publisher(Twist, '/obstacle2/cmd_vel', 10)
        self.obstacle3_pub = self.create_publisher(Twist, '/obstacle3/cmd_vel', 10)
        
        # Control timers
        self.obstacle1_timer = self.create_timer(0.1, self.control_obstacle1)  # 10 Hz
        self.obstacle2_timer = self.create_timer(0.1, self.control_obstacle2)  # 10 Hz
        self.obstacle3_timer = self.create_timer(0.1, self.control_obstacle3)  # 10 Hz
        
        # Movement parameters
        self.time_start = time.time()
        self.obstacle1_speed = 2.0  # m/s
        self.obstacle2_speed = 1.5  # m/s
        self.obstacle3_speed = 1.0  # m/s
        
        # Movement ranges
        self.obstacle1_range = 8.0  # Horizontal movement range
        self.obstacle2_range = 6.0  # Vertical movement range
        self.obstacle3_radius = 3.0  # Circular movement radius
        
        self.get_logger().info("🚧 Moving Obstacles Controller started!")
        self.get_logger().info("   Obstacle 1: Horizontal patrol (orange)")
        self.get_logger().info("   Obstacle 2: Vertical patrol (blue)")
        self.get_logger().info("   Obstacle 3: Circular pattern (purple)")
    
    def control_obstacle1(self):
        """Control obstacle 1: Horizontal patrol."""
        current_time = time.time() - self.time_start
        
        # Create horizontal back-and-forth movement
        x_pos = self.obstacle1_range * math.sin(current_time * 0.5)
        x_vel = self.obstacle1_speed * math.cos(current_time * 0.5)
        
        cmd = Twist()
        cmd.linear.x = x_vel
        cmd.linear.y = 0.0
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        
        self.obstacle1_pub.publish(cmd)
    
    def control_obstacle2(self):
        """Control obstacle 2: Vertical patrol."""
        current_time = time.time() - self.time_start
        
        # Create vertical back-and-forth movement
        y_pos = self.obstacle2_range * math.sin(current_time * 0.3)
        y_vel = self.obstacle2_speed * math.cos(current_time * 0.3)
        
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = y_vel
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        
        self.obstacle2_pub.publish(cmd)
    
    def control_obstacle3(self):
        """Control obstacle 3: Circular pattern."""
        current_time = time.time() - self.time_start
        
        # Create circular movement
        angle = current_time * 0.4
        radius = self.obstacle3_radius
        
        # Calculate velocity components for circular motion
        x_vel = -self.obstacle3_speed * math.sin(angle)
        y_vel = self.obstacle3_speed * math.cos(angle)
        
        cmd = Twist()
        cmd.linear.x = x_vel
        cmd.linear.y = y_vel
        cmd.linear.z = 0.0
        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = 0.0
        
        self.obstacle3_pub.publish(cmd)
    
    def stop_all_obstacles(self):
        """Stop all moving obstacles."""
        stop_cmd = Twist()
        
        self.obstacle1_pub.publish(stop_cmd)
        self.obstacle2_pub.publish(stop_cmd)
        self.obstacle3_pub.publish(stop_cmd)
        
        self.get_logger().info("🛑 All obstacles stopped!")

def main(args=None):
    rclpy.init(args=args)
    
    controller = MovingObstaclesController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("🛑 Stopping moving obstacles...")
        controller.stop_all_obstacles()
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

