#!/usr/bin/env python3
"""
Coordinate system transformations and geometric utilities for the swarm system.
Handles ENU/NED conversions, quaternion operations, and safety calculations.
"""

import numpy as np
import math
from typing import Tuple, List, Optional
from geometry_msgs.msg import Point, Quaternion, Pose, Twist
from nav_msgs.msg import Odometry


class TransformUtils:
    """Utility class for coordinate transformations and geometric calculations."""
    
    @staticmethod
    def enu_to_ned_position(enu_pos: Point) -> Point:
        """
        Convert ENU (East-North-Up) position to NED (North-East-Down).
        
        Args:
            enu_pos: Position in ENU coordinates
            
        Returns:
            Position in NED coordinates
        """
        ned_pos = Point()
        ned_pos.x = enu_pos.y   # North = East
        ned_pos.y = enu_pos.x   # East = North  
        ned_pos.z = -enu_pos.z  # Down = -Up
        return ned_pos
    
    @staticmethod
    def ned_to_enu_position(ned_pos: Point) -> Point:
        """
        Convert NED (North-East-Down) position to ENU (East-North-Up).
        
        Args:
            ned_pos: Position in NED coordinates
            
        Returns:
            Position in ENU coordinates
        """
        enu_pos = Point()
        enu_pos.x = ned_pos.y   # East = North
        enu_pos.y = ned_pos.x   # North = East
        enu_pos.z = -ned_pos.z  # Up = -Down
        return enu_pos
    
    @staticmethod
    def enu_to_ned_velocity(enu_vel: Point) -> Point:
        """
        Convert ENU velocity to NED velocity.
        
        Args:
            enu_vel: Velocity in ENU coordinates
            
        Returns:
            Velocity in NED coordinates
        """
        ned_vel = Point()
        ned_vel.x = enu_vel.y   # North = East
        ned_vel.y = enu_vel.x   # East = North
        ned_vel.z = -enu_vel.z  # Down = -Up
        return ned_vel
    
    @staticmethod
    def ned_to_enu_velocity(ned_vel: Point) -> Point:
        """
        Convert NED velocity to ENU velocity.
        
        Args:
            ned_vel: Velocity in NED coordinates
            
        Returns:
            Velocity in ENU coordinates
        """
        enu_vel = Point()
        enu_vel.x = ned_vel.y   # East = North
        enu_vel.y = ned_vel.x   # North = East
        enu_vel.z = -ned_vel.z  # Up = -Down
        return enu_vel
    
    @staticmethod
    def quaternion_to_euler(quat: Quaternion) -> Tuple[float, float, float]:
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).
        
        Args:
            quat: Quaternion message
            
        Returns:
            Tuple of (roll, pitch, yaw) in radians
        """
        # Convert to numpy array for easier calculation
        q = np.array([quat.x, quat.y, quat.z, quat.w])
        
        # Normalize quaternion
        q = q / np.linalg.norm(q)
        
        # Extract Euler angles
        roll = math.atan2(2 * (q[3] * q[0] + q[1] * q[2]), 
                          1 - 2 * (q[0] * q[0] + q[1] * q[1]))
        pitch = math.asin(2 * (q[3] * q[1] - q[2] * q[0]))
        yaw = math.atan2(2 * (q[3] * q[2] + q[0] * q[1]), 
                         1 - 2 * (q[1] * q[1] + q[2] * q[2]))
        
        return roll, pitch, yaw
    
    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
        """
        Convert Euler angles to quaternion.
        
        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians
            
        Returns:
            Quaternion message
        """
        # Convert to quaternion using half-angle formulas
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        quat = Quaternion()
        quat.w = cy * cp * cr + sy * sp * sr
        quat.x = cy * cp * sr - sy * sp * cr
        quat.y = sy * cp * cr + cy * sp * sr
        quat.z = sy * cp * cr - cy * sp * sr
        
        return quat
    
    @staticmethod
    def calculate_distance(pos1: Point, pos2: Point) -> float:
        """
        Calculate Euclidean distance between two 3D points.
        
        Args:
            pos1: First position
            pos2: Second position
            
        Returns:
            Distance in meters
        """
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    @staticmethod
    def calculate_2d_distance(pos1: Point, pos2: Point) -> float:
        """
        Calculate 2D Euclidean distance between two points (ignoring Z).
        
        Args:
            pos1: First position
            pos2: Second position
            
        Returns:
            Distance in meters (2D)
        """
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        return math.sqrt(dx*dx + dy*dy)
    
    @staticmethod
    def calculate_heading(from_pos: Point, to_pos: Point) -> float:
        """
        Calculate heading angle from one position to another.
        
        Args:
            from_pos: Starting position
            to_pos: Target position
            
        Returns:
            Heading angle in radians (-π to π)
        """
        dx = to_pos.x - from_pos.x
        dy = to_pos.y - from_pos.y
        return math.atan2(dy, dx)
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to range [-π, π].
        
        Args:
            angle: Input angle in radians
            
        Returns:
            Normalized angle in radians
        """
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    @staticmethod
    def is_within_bounds(position: Point, bounds: dict) -> bool:
        """
        Check if position is within specified bounds.
        
        Args:
            position: Position to check
            bounds: Dictionary with x_min, x_max, y_min, y_max, z_min, z_max
            
        Returns:
            True if within bounds, False otherwise
        """
        return (bounds['x_min'] <= position.x <= bounds['x_max'] and
                bounds['y_min'] <= position.y <= bounds['y_max'] and
                bounds['z_min'] <= position.z <= bounds['z_max'])
    
    @staticmethod
    def calculate_safety_margin(separation: float, safety_radius: float) -> float:
        """
        Calculate safety margin based on current separation and safety radius.
        
        Args:
            separation: Current separation distance
            safety_radius: Required safety radius
            
        Returns:
            Safety margin (positive = safe, negative = unsafe)
        """
        return separation - safety_radius


class SafetyUtils:
    """Utility class for safety-related calculations."""
    
    @staticmethod
    def check_collision_risk(pos1: Point, vel1: Point, 
                           pos2: Point, vel2: Point,
                           safety_radius: float, time_horizon: float) -> bool:
        """
        Check if there's a collision risk between two moving objects.
        
        Args:
            pos1: Position of first object
            vel1: Velocity of first object
            pos2: Position of second object
            vel2: Velocity of second object
            safety_radius: Minimum safe separation
            time_horizon: Time horizon for prediction
            
        Returns:
            True if collision risk detected, False otherwise
        """
        # Calculate relative position and velocity
        rel_pos = Point()
        rel_pos.x = pos2.x - pos1.x
        rel_pos.y = pos2.y - pos1.y
        rel_pos.z = pos2.z - pos1.z
        
        rel_vel = Point()
        rel_vel.x = vel2.x - vel1.x
        rel_vel.y = vel2.y - vel1.y
        rel_vel.z = vel2.z - vel1.z
        
        # Calculate closest approach distance
        rel_pos_mag = TransformUtils.calculate_distance(rel_pos, Point())
        rel_vel_mag = TransformUtils.calculate_distance(rel_vel, Point())
        
        if rel_vel_mag < 1e-6:  # Nearly stationary relative to each other
            return rel_pos_mag < safety_radius
        
        # Calculate time to closest approach
        t_ca = -np.dot([rel_pos.x, rel_pos.y, rel_pos.z], 
                       [rel_vel.x, rel_vel.y, rel_vel.z]) / (rel_vel_mag * rel_vel_mag)
        
        # Clamp to time horizon
        t_ca = max(0.0, min(t_ca, time_horizon))
        
        # Calculate closest approach distance
        d_ca = math.sqrt(rel_pos_mag * rel_pos_mag - 
                         (t_ca * rel_vel_mag) * (t_ca * rel_vel_mag))
        
        return d_ca < safety_radius
    
    @staticmethod
    def calculate_emergency_stop_velocity(current_vel: Point, 
                                        max_decel: float) -> Point:
        """
        Calculate velocity for emergency stop.
        
        Args:
            current_vel: Current velocity
            max_decel: Maximum deceleration
            
        Returns:
            Target velocity for emergency stop
        """
        stop_vel = Point()
        vel_mag = TransformUtils.calculate_distance(current_vel, Point())
        
        if vel_mag < 0.1:  # Already nearly stopped
            return Point()
        
        # Calculate deceleration time
        t_stop = vel_mag / max_decel
        
        # Apply deceleration
        stop_vel.x = current_vel.x * (1.0 - 1.0 / t_stop)
        stop_vel.y = current_vel.y * (1.0 - 1.0 / t_stop)
        stop_vel.z = current_vel.z * (1.0 - 1.0 / t_stop)
        
        return stop_vel
