#!/usr/bin/env python3
"""
Tests for the transforms utility functions.
"""

import pytest
import math
from geometry_msgs.msg import Point, Quaternion
from swarm_core.utils.transforms import TransformUtils, SafetyUtils


class TestTransformUtils:
    """Test cases for TransformUtils class."""
    
    def test_enu_to_ned_position(self):
        """Test ENU to NED position conversion."""
        enu_pos = Point(x=1.0, y=2.0, z=3.0)
        ned_pos = TransformUtils.enu_to_ned_position(enu_pos)
        
        assert ned_pos.x == 2.0  # North = East
        assert ned_pos.y == 1.0  # East = North
        assert ned_pos.z == -3.0  # Down = -Up
    
    def test_ned_to_enu_position(self):
        """Test NED to ENU position conversion."""
        ned_pos = Point(x=1.0, y=2.0, z=3.0)
        enu_pos = TransformUtils.ned_to_enu_position(ned_pos)
        
        assert enu_pos.x == 2.0  # East = North
        assert enu_pos.y == 1.0  # North = East
        assert enu_pos.z == -3.0  # Up = -Down
    
    def test_calculate_distance(self):
        """Test distance calculation between two points."""
        pos1 = Point(x=0.0, y=0.0, z=0.0)
        pos2 = Point(x=3.0, y=4.0, z=0.0)
        
        distance = TransformUtils.calculate_distance(pos1, pos2)
        assert distance == 5.0  # 3-4-5 triangle
    
    def test_calculate_2d_distance(self):
        """Test 2D distance calculation."""
        pos1 = Point(x=0.0, y=0.0, z=10.0)
        pos2 = Point(x=3.0, y=4.0, z=20.0)
        
        distance = TransformUtils.calculate_2d_distance(pos1, pos2)
        assert distance == 5.0  # Should ignore Z coordinate
    
    def test_calculate_heading(self):
        """Test heading calculation."""
        from_pos = Point(x=0.0, y=0.0, z=0.0)
        to_pos = Point(x=1.0, y=1.0, z=0.0)
        
        heading = TransformUtils.calculate_heading(from_pos, to_pos)
        assert abs(heading - math.pi/4) < 1e-6  # 45 degrees
    
    def test_normalize_angle(self):
        """Test angle normalization."""
        # Test angles that are already normalized
        assert TransformUtils.normalize_angle(0.0) == 0.0
        assert TransformUtils.normalize_angle(math.pi) == math.pi
        assert TransformUtils.normalize_angle(-math.pi) == -math.pi
        
        # Test angles that need normalization
        assert abs(TransformUtils.normalize_angle(2*math.pi)) < 1e-6
        assert abs(TransformUtils.normalize_angle(-2*math.pi)) < 1e-6
        assert abs(TransformUtils.normalize_angle(3*math.pi) - math.pi) < 1e-6
    
    def test_is_within_bounds(self):
        """Test bounds checking."""
        bounds = {
            'x_min': -10.0, 'x_max': 10.0,
            'y_min': -5.0, 'y_max': 5.0,
            'z_min': 0.0, 'z_max': 20.0
        }
        
        # Test point within bounds
        pos = Point(x=0.0, y=0.0, z=10.0)
        assert TransformUtils.is_within_bounds(pos, bounds)
        
        # Test point outside bounds
        pos = Point(x=15.0, y=0.0, z=10.0)
        assert not TransformUtils.is_within_bounds(pos, bounds)
    
    def test_calculate_safety_margin(self):
        """Test safety margin calculation."""
        separation = 3.0
        safety_radius = 1.5
        
        margin = TransformUtils.calculate_safety_margin(separation, safety_radius)
        assert margin == 1.5  # 3.0 - 1.5 = 1.5


class TestSafetyUtils:
    """Test cases for SafetyUtils class."""
    
    def test_check_collision_risk_stationary(self):
        """Test collision risk check for stationary objects."""
        pos1 = Point(x=0.0, y=0.0, z=0.0)
        vel1 = Point(x=0.0, y=0.0, z=0.0)
        pos2 = Point(x=1.0, y=0.0, z=0.0)
        vel2 = Point(x=0.0, y=0.0, z=0.0)
        
        # No collision risk when separation > safety radius
        risk = SafetyUtils.check_collision_risk(pos1, vel1, pos2, vel2, 0.5, 2.0)
        assert not risk
        
        # Collision risk when separation < safety radius
        risk = SafetyUtils.check_collision_risk(pos1, vel1, pos2, vel2, 1.5, 2.0)
        assert risk
    
    def test_check_collision_risk_moving(self):
        """Test collision risk check for moving objects."""
        pos1 = Point(x=0.0, y=0.0, z=0.0)
        vel1 = Point(x=1.0, y=0.0, z=0.0)
        pos2 = Point(x=5.0, y=0.0, z=0.0)
        vel2 = Point(x=-1.0, y=0.0, z=0.0)
        
        # Collision risk when objects are moving toward each other
        risk = SafetyUtils.check_collision_risk(pos1, vel1, pos2, vel2, 1.0, 5.0)
        assert risk
    
    def test_calculate_emergency_stop_velocity(self):
        """Test emergency stop velocity calculation."""
        current_vel = Point(x=2.0, y=1.0, z=0.0)
        max_decel = 2.0
        
        stop_vel = SafetyUtils.calculate_emergency_stop_velocity(current_vel, max_decel)
        
        # Should reduce velocity magnitude
        vel_mag = TransformUtils.calculate_distance(stop_vel, Point())
        current_mag = TransformUtils.calculate_distance(current_vel, Point())
        assert vel_mag < current_mag


if __name__ == '__main__':
    pytest.main([__file__])
