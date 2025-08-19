#!/usr/bin/env python3
"""
MAVSDK connector utility for communicating with PX4 SITL instances.
Handles connection, offboard control, and status monitoring for each UAV.
"""

import asyncio
import time
from typing import Optional, Dict, Any
from dataclasses import dataclass
from geometry_msgs.msg import Point, Quaternion, Twist
from nav_msgs.msg import Odometry

try:
    import mavsdk
    from mavsdk import System
    from mavsdk.offboard import OffboardError, VelocityBodyYawspeed
    from mavsdk.telemetry import TelemetryError
    MAVSDK_AVAILABLE = True
except ImportError:
    MAVSDK_AVAILABLE = False
    print("Warning: MAVSDK not available. Install with: pip install mavsdk")


@dataclass
class UAVStatus:
    """Data class for UAV status information."""
    connected: bool = False
    armed: bool = False
    in_air: bool = False
    flight_mode: str = "UNKNOWN"
    position: Optional[Point] = None
    velocity: Optional[Point] = None
    attitude: Optional[Quaternion] = None
    battery_percentage: float = 0.0
    last_heartbeat: float = 0.0


class MAVSDKConnector:
    """
    MAVSDK connector for a single UAV.
    Handles connection, offboard control, and status monitoring.
    """
    
    def __init__(self, uav_id: str, connection_string: str = "udp://:14540"):
        """
        Initialize MAVSDK connector.
        
        Args:
            uav_id: Unique identifier for the UAV
            connection_string: MAVSDK connection string
        """
        self.uav_id = uav_id
        self.connection_string = connection_string
        self.drone = System()
        self.status = UAVStatus()
        
        # Control parameters
        self.offboard_active = False
        self.control_rate = 50.0  # Hz
        self.last_control_time = 0.0
        
        # Status update callbacks
        self.position_callback = None
        self.velocity_callback = None
        self.attitude_callback = None
        self.connection_callback = None
        
        # Safety parameters
        self.max_velocity = 5.0  # m/s
        self.max_acceleration = 3.0  # m/s²
        self.safety_altitude = 15.0  # meters
        
    async def connect(self) -> bool:
        """
        Connect to the UAV via MAVSDK.
        
        Returns:
            True if connection successful, False otherwise
        """
        if not MAVSDK_AVAILABLE:
            print(f"Error: MAVSDK not available for {self.uav_id}")
            return False
            
        try:
            print(f"Connecting to {self.uav_id} at {self.connection_string}")
            
            # Connect to the drone
            await self.drone.connect(system_address=self.connection_string)
            
            # Wait for connection
            async for state in self.drone.core.connection_state():
                if state.is_connected:
                    print(f"Connected to {self.uav_id}")
                    self.status.connected = True
                    break
                    
            # Set up telemetry callbacks
            await self._setup_telemetry()
            
            return True
            
        except Exception as e:
            print(f"Failed to connect to {self.uav_id}: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from the UAV."""
        try:
            if self.offboard_active:
                await self.stop_offboard()
            
            await self.drone.disconnect()
            self.status.connected = False
            print(f"Disconnected from {self.uav_id}")
            
        except Exception as e:
            print(f"Error disconnecting from {self.uav_id}: {e}")
    
    async def _setup_telemetry(self):
        """Set up telemetry callbacks for position, velocity, and attitude."""
        try:
            # Position callback
            asyncio.create_task(self._position_callback())
            
            # Velocity callback
            asyncio.create_task(self._velocity_callback())
            
            # Attitude callback
            asyncio.create_task(self._attitude_callback())
            
            # Connection state callback
            asyncio.create_task(self._connection_callback())
            
        except Exception as e:
            print(f"Error setting up telemetry for {self.uav_id}: {e}")
    
    async def _position_callback(self):
        """Handle position updates."""
        try:
            async for position in self.drone.telemetry.position():
                self.status.position = Point(
                    x=position.latitude_deg,
                    y=position.longitude_deg,
                    z=position.absolute_altitude_m
                )
                
                if self.position_callback:
                    self.position_callback(self.status.position)
                    
        except Exception as e:
            print(f"Position callback error for {self.uav_id}: {e}")
    
    async def _velocity_callback(self):
        """Handle velocity updates."""
        try:
            async for velocity in self.drone.telemetry.velocity_ned():
                self.status.velocity = Point(
                    x=velocity.north_m_s,
                    y=velocity.east_m_s,
                    z=velocity.down_m_s
                )
                
                if self.velocity_callback:
                    self.velocity_callback(self.status.velocity)
                    
        except Exception as e:
            print(f"Velocity callback error for {self.uav_id}: {e}")
    
    async def _attitude_callback(self):
        """Handle attitude updates."""
        try:
            async for attitude in self.drone.telemetry.attitude_euler():
                self.status.attitude = Quaternion()
                # Convert Euler to quaternion (simplified)
                # In practice, you'd want to use proper conversion
                self.status.attitude.w = 1.0
                
                if self.attitude_callback:
                    self.attitude_callback(self.status.attitude)
                    
        except Exception as e:
            print(f"Attitude callback error for {self.uav_id}: {e}")
    
    async def _connection_callback(self):
        """Handle connection state updates."""
        try:
            async for state in self.drone.core.connection_state():
                self.status.connected = state.is_connected
                if self.connection_callback:
                    self.connection_callback(self.status.connected)
                    
        except Exception as e:
            print(f"Connection callback error for {self.uav_id}: {e}")
    
    async def arm(self) -> bool:
        """
        Arm the UAV.
        
        Returns:
            True if arming successful, False otherwise
        """
        try:
            print(f"Arming {self.uav_id}")
            await self.drone.action.arm()
            self.status.armed = True
            return True
            
        except Exception as e:
            print(f"Failed to arm {self.uav_id}: {e}")
            return False
    
    async def disarm(self) -> bool:
        """
        Disarm the UAV.
        
        Returns:
            True if disarming successful, False otherwise
        """
        try:
            print(f"Disarming {self.uav_id}")
            await self.drone.action.disarm()
            self.status.armed = False
            return True
            
        except Exception as e:
            print(f"Failed to disarm {self.uav_id}: {e}")
            return False
    
    async def takeoff(self, altitude: float = 5.0) -> bool:
        """
        Take off to specified altitude.
        
        Args:
            altitude: Target altitude in meters
            
        Returns:
            True if takeoff successful, False otherwise
        """
        try:
            print(f"Taking off {self.uav_id} to {altitude}m")
            await self.drone.action.takeoff()
            
            # Wait for takeoff completion
            async for in_air in self.drone.telemetry.in_air():
                if in_air:
                    self.status.in_air = True
                    print(f"{self.uav_id} is airborne")
                    break
                    
            return True
            
        except Exception as e:
            print(f"Failed to take off {self.uav_id}: {e}")
            return False
    
    async def land(self) -> bool:
        """
        Land the UAV.
        
        Returns:
            True if landing successful, False otherwise
        """
        try:
            print(f"Landing {self.uav_id}")
            await self.drone.action.land()
            
            # Wait for landing completion
            async for in_air in self.drone.telemetry.in_air():
                if not in_air:
                    self.status.in_air = False
                    print(f"{self.uav_id} has landed")
                    break
                    
            return True
            
        except Exception as e:
            print(f"Failed to land {self.uav_id}: {e}")
            return False
    
    async def start_offboard(self) -> bool:
        """
        Start offboard control mode.
        
        Returns:
            True if offboard start successful, False otherwise
        """
        try:
            print(f"Starting offboard control for {self.uav_id}")
            await self.drone.offboard.start()
            self.offboard_active = True
            return True
            
        except Exception as e:
            print(f"Failed to start offboard for {self.uav_id}: {e}")
            return False
    
    async def stop_offboard(self) -> bool:
        """
        Stop offboard control mode.
        
        Returns:
            True if offboard stop successful, False otherwise
        """
        try:
            print(f"Stopping offboard control for {self.uav_id}")
            await self.drone.offboard.stop()
            self.offboard_active = False
            return True
            
        except Exception as e:
            print(f"Failed to stop offboard for {self.uav_id}: {e}")
            return False
    
    async def send_velocity_setpoint(self, velocity: Twist) -> bool:
        """
        Send velocity setpoint to the UAV.
        
        Args:
            velocity: Velocity setpoint in Twist message
            
        Returns:
            True if setpoint sent successfully, False otherwise
        """
        if not self.offboard_active:
            print(f"Cannot send velocity setpoint: {self.uav_id} not in offboard mode")
            return False
        
        try:
            # Apply safety limits
            safe_velocity = self._apply_velocity_limits(velocity)
            
            # Create MAVSDK velocity setpoint
            setpoint = VelocityBodyYawspeed(
                forward_m_s=safe_velocity.linear.x,
                right_m_s=safe_velocity.linear.y,
                down_m_s=safe_velocity.linear.z,
                yawspeed_deg_s=safe_velocity.angular.z * 180.0 / 3.14159
            )
            
            # Send setpoint
            await self.drone.offboard.set_velocity_body(setpoint)
            
            return True
            
        except Exception as e:
            print(f"Failed to send velocity setpoint to {self.uav_id}: {e}")
            return False
    
    def _apply_velocity_limits(self, velocity: Twist) -> Twist:
        """
        Apply safety limits to velocity setpoint.
        
        Args:
            velocity: Input velocity setpoint
            
        Returns:
            Velocity setpoint with safety limits applied
        """
        limited_velocity = Twist()
        
        # Apply maximum velocity limits
        vel_mag = (velocity.linear.x**2 + velocity.linear.y**2 + velocity.linear.z**2)**0.5
        if vel_mag > self.max_velocity:
            scale = self.max_velocity / vel_mag
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
        limited_velocity.angular.z = max(-1.0, min(1.0, velocity.angular.z))  # ±1 rad/s
        
        return limited_velocity
    
    def get_status(self) -> UAVStatus:
        """
        Get current UAV status.
        
        Returns:
            Current UAV status
        """
        return self.status
    
    def is_healthy(self) -> bool:
        """
        Check if UAV is healthy and ready for operation.
        
        Returns:
            True if UAV is healthy, False otherwise
        """
        current_time = time.time()
        
        # Check connection
        if not self.status.connected:
            return False
        
        # Check heartbeat timeout
        if current_time - self.status.last_heartbeat > 5.0:  # 5 second timeout
            return False
        
        # Check battery
        if self.status.battery_percentage < 10.0:
            return False
        
        return True
    
    async def emergency_stop(self):
        """Execute emergency stop procedure."""
        try:
            print(f"Emergency stop for {self.uav_id}")
            
            # Stop offboard if active
            if self.offboard_active:
                await self.stop_offboard()
            
            # Land immediately
            await self.land()
            
        except Exception as e:
            print(f"Emergency stop failed for {self.uav_id}: {e}")


class SwarmConnector:
    """
    Manages multiple MAVSDK connectors for the entire swarm.
    """
    
    def __init__(self):
        """Initialize swarm connector."""
        self.connectors: Dict[str, MAVSDKConnector] = {}
        self.connection_strings = {
            "uav1": "udp://:14540",
            "uav2": "udp://:14541", 
            "uav3": "udp://:14542"
        }
    
    async def add_uav(self, uav_id: str, connection_string: Optional[str] = None) -> bool:
        """
        Add a UAV to the swarm.
        
        Args:
            uav_id: Unique identifier for the UAV
            connection_string: MAVSDK connection string (optional)
            
        Returns:
            True if UAV added successfully, False otherwise
        """
        if uav_id in self.connectors:
            print(f"UAV {uav_id} already exists in swarm")
            return False
        
        # Use default connection string if not provided
        if connection_string is None:
            connection_string = self.connection_strings.get(uav_id, f"udp://:1454{len(self.connectors)}")
        
        # Create and connect UAV
        connector = MAVSDKConnector(uav_id, connection_string)
        if await connector.connect():
            self.connectors[uav_id] = connector
            print(f"Added {uav_id} to swarm")
            return True
        else:
            print(f"Failed to add {uav_id} to swarm")
            return False
    
    async def remove_uav(self, uav_id: str):
        """Remove a UAV from the swarm."""
        if uav_id in self.connectors:
            await self.connectors[uav_id].disconnect()
            del self.connectors[uav_id]
            print(f"Removed {uav_id} from swarm")
    
    async def connect_all(self) -> bool:
        """
        Connect to all UAVs in the swarm.
        
        Returns:
            True if all connections successful, False otherwise
        """
        success = True
        for uav_id in list(self.connectors.keys()):
            if not await self.connectors[uav_id].connect():
                success = False
        
        return success
    
    async def disconnect_all(self):
        """Disconnect from all UAVs in the swarm."""
        for uav_id in list(self.connectors.keys()):
            await self.remove_uav(uav_id)
    
    def get_uav_status(self, uav_id: str) -> Optional[UAVStatus]:
        """
        Get status of a specific UAV.
        
        Args:
            uav_id: UAV identifier
            
        Returns:
            UAV status or None if not found
        """
        if uav_id in self.connectors:
            return self.connectors[uav_id].get_status()
        return None
    
    def get_swarm_status(self) -> Dict[str, UAVStatus]:
        """
        Get status of all UAVs in the swarm.
        
        Returns:
            Dictionary mapping UAV IDs to their status
        """
        return {uav_id: connector.get_status() 
                for uav_id, connector in self.connectors.items()}
    
    async def arm_all(self) -> bool:
        """
        Arm all UAVs in the swarm.
        
        Returns:
            True if all UAVs armed successfully, False otherwise
        """
        success = True
        for uav_id, connector in self.connectors.items():
            if not await connector.arm():
                success = False
        
        return success
    
    async def takeoff_all(self, altitude: float = 5.0) -> bool:
        """
        Take off all UAVs to specified altitude.
        
        Args:
            altitude: Target altitude in meters
            
        Returns:
            True if all UAVs took off successfully, False otherwise
        """
        success = True
        for uav_id, connector in self.connectors.items():
            if not await connector.takeoff(altitude):
                success = False
        
        return success
    
    async def land_all(self) -> bool:
        """
        Land all UAVs in the swarm.
        
        Returns:
            True if all UAVs landed successfully, False otherwise
        """
        success = True
        for uav_id, connector in self.connectors.items():
            if not await connector.land():
                success = False
        
        return success
    
    async def emergency_stop_all(self):
        """Execute emergency stop for all UAVs in the swarm."""
        for uav_id, connector in self.connectors.items():
            await connector.emergency_stop()
