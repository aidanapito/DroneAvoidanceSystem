#!/usr/bin/env python3
"""
Test Coordinator for Placeholder UAVs

This script tests the basic coordinator control logic with placeholder UAVs
without requiring full ROS 2 or PX4 SITL.
"""

import asyncio
import time
import json
import socket
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum


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
    position: Tuple[float, float, float]
    velocity: Optional[Tuple[float, float, float]] = None
    priority: int = 1
    timestamp: float = 0.0
    completed: bool = False


class PlaceholderUAVConnector:
    """Connector for placeholder UAVs via UDP."""
    
    def __init__(self, uav_id: str, port: int):
        self.uav_id = uav_id
        self.port = port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(0.1)
        self.connected = False
        
        # UAV state
        self.armed = False
        self.in_air = False
        self.position = (0.0, 0.0, 0.0)
        self.velocity = (0.0, 0.0, 0.0)
        self.flight_mode = 0
        
    async def connect(self) -> bool:
        """Connect to the placeholder UAV."""
        try:
            # Test connection by sending a status request
            await self._send_command('get_status')
            
            # Wait for response
            try:
                response = await self._receive_response(timeout=2.0)
                if response and response.get('command') == 'status_response':
                    self.connected = True
                    print(f"✅ Connected to {self.uav_id}")
                    return True
            except asyncio.TimeoutError:
                pass
            
            print(f"❌ Failed to connect to {self.uav_id}")
            return False
            
        except Exception as e:
            print(f"Error connecting to {self.uav_id}: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from the UAV."""
        self.connected = False
        if self.socket:
            self.socket.close()
    
    async def arm(self) -> bool:
        """Arm the UAV."""
        if not self.connected:
            return False
        
        try:
            await self._send_command('arm')
            response = await self._receive_response(timeout=2.0)
            
            if response and response.get('command') == 'arm_response':
                success = response.get('success', False)
                if success:
                    self.armed = True
                    print(f"🔒 {self.uav_id} armed")
                return success
            
            return False
            
        except Exception as e:
            print(f"Error arming {self.uav_id}: {e}")
            return False
    
    async def takeoff(self, altitude: float = 5.0) -> bool:
        """Take off to specified altitude."""
        if not self.connected or not self.armed:
            return False
        
        try:
            await self._send_command('takeoff', {'altitude': altitude})
            response = await self._receive_response(timeout=5.0)
            
            if response and response.get('command') == 'takeoff_response':
                success = response.get('success', False)
                if success:
                    self.in_air = True
                    print(f"🚀 {self.uav_id} taking off to {altitude}m")
                return success
            
            return False
            
        except Exception as e:
            print(f"Error taking off {self.uav_id}: {e}")
            return False
    
    async def land(self) -> bool:
        """Land the UAV."""
        if not self.connected or not self.in_air:
            return False
        
        try:
            await self._send_command('land')
            response = await self._receive_response(timeout=5.0)
            
            if response and response.get('command') == 'land_response':
                success = response.get('success', False)
                if success:
                    self.in_air = False
                    print(f"🛬 {self.uav_id} landing")
                return success
            
            return False
            
        except Exception as e:
            print(f"Error landing {self.uav_id}: {e}")
            return False
    
    async def set_position(self, x: float, y: float, z: float) -> bool:
        """Set target position for the UAV."""
        if not self.connected or not self.in_air:
            return False
        
        try:
            await self._send_command('set_position', {'x': x, 'y': y, 'z': z})
            return True
            
        except Exception as e:
            print(f"Error setting position for {self.uav_id}: {e}")
            return False
    
    async def get_status(self) -> Optional[dict]:
        """Get current UAV status."""
        if not self.connected:
            return None
        
        try:
            await self._send_command('get_status')
            response = await self._receive_response(timeout=1.0)
            
            if response and response.get('command') == 'status_response':
                # Update local state
                self.armed = response.get('armed', False)
                self.in_air = response.get('in_air', False)
                self.position = response.get('position', (0.0, 0.0, 0.0))
                self.velocity = response.get('velocity', (0.0, 0.0, 0.0))
                self.flight_mode = response.get('flight_mode', 0)
                
                return response
            
            return None
            
        except Exception as e:
            print(f"Error getting status for {self.uav_id}: {e}")
            return None
    
    async def _send_command(self, command: str, params: dict = None):
        """Send a command to the UAV."""
        message = {
            'command': command,
            'uav_id': self.uav_id
        }
        
        if params:
            message.update(params)
        
        try:
            data = json.dumps(message).encode('utf-8')
            self.socket.sendto(data, ('localhost', self.port))
        except Exception as e:
            print(f"Error sending command to {self.uav_id}: {e}")
    
    async def _receive_response(self, timeout: float = 1.0) -> Optional[dict]:
        """Receive a response from the UAV."""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                data, addr = self.socket.recvfrom(1024)
                message = data.decode('utf-8')
                
                if message.startswith('{'):
                    response = json.loads(message)
                    if response.get('uav_id') == self.uav_id:
                        return response
                        
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error receiving response from {self.uav_id}: {e}")
        
        return None


class TestCoordinator:
    """Test coordinator for placeholder UAVs."""
    
    def __init__(self):
        self.mission_state = MissionState.IDLE
        self.uav_connectors: Dict[str, PlaceholderUAVConnector] = {}
        self.uav_goals: Dict[str, UAVGoal] = {}
        self.mission_start_time = 0.0
        self.mission_timeout = 120.0  # 2 minutes
        
        # Formation positions
        self.formation_positions = [
            (0.0, 0.0, 5.0),
            (3.0, 0.0, 5.0),
            (0.0, 3.0, 5.0)
        ]
        
        # Control parameters
        self.control_rate = 10.0  # Hz
        self.position_tolerance = 0.5
        self.takeoff_height = 5.0
        
    async def initialize(self):
        """Initialize the coordinator and connect to UAVs."""
        print("🚁 Initializing Test Coordinator...")
        
        # Create UAV connectors
        ports = [14540, 14541, 14542]
        for i, port in enumerate(ports):
            uav_id = f"uav{i+1}"
            connector = PlaceholderUAVConnector(uav_id, port)
            self.uav_connectors[uav_id] = connector
        
        # Connect to all UAVs
        print("🔌 Connecting to UAVs...")
        for uav_id, connector in self.uav_connectors.items():
            if await connector.connect():
                print(f"   ✅ {uav_id} connected")
            else:
                print(f"   ❌ {uav_id} failed to connect")
        
        # Check if we have at least one UAV connected
        connected_uavs = [uav_id for uav_id, connector in self.uav_connectors.items() 
                         if connector.connected]
        
        if not connected_uavs:
            print("❌ No UAVs connected. Cannot proceed.")
            return False
        
        print(f"✅ Connected to {len(connected_uavs)} UAVs: {', '.join(connected_uavs)}")
        return True
    
    async def start_mission(self):
        """Start the test mission."""
        if self.mission_state != MissionState.IDLE:
            print("⚠️  Mission already in progress")
            return False
        
        print("🚀 Starting test mission...")
        self.mission_state = MissionState.INITIALIZING
        self.mission_start_time = time.time()
        return True
    
    async def stop_mission(self):
        """Stop the test mission."""
        if self.mission_state == MissionState.IDLE:
            print("⚠️  No mission in progress")
            return False
        
        print("🛑 Stopping test mission...")
        self.mission_state = MissionState.LANDING
        return True
    
    async def run_control_loop(self):
        """Main control loop."""
        print("🔄 Starting control loop...")
        
        last_control_time = 0.0
        
        while self.mission_state != MissionState.COMPLETED:
            current_time = time.time()
            
            # Check mission timeout
            if (self.mission_state != MissionState.IDLE and 
                current_time - self.mission_start_time > self.mission_timeout):
                print("⏰ Mission timeout reached")
                self.mission_state = MissionState.COMPLETED
                break
            
            # Control loop timing
            if current_time - last_control_time >= 1.0 / self.control_rate:
                await self._update_mission_state()
                await self._execute_mission_state()
                await self._check_safety_conditions()
                await self._publish_status()
                
                last_control_time = current_time
            
            await asyncio.sleep(0.01)  # Small sleep
        
        print("✅ Control loop completed")
    
    async def _update_mission_state(self):
        """Update mission state based on current conditions."""
        if self.mission_state == MissionState.INITIALIZING:
            # Check if all UAVs are connected and ready
            if await self._all_uavs_ready():
                self.mission_state = MissionState.TAKEOFF
                print("🔄 All UAVs ready, transitioning to takeoff")
                
        elif self.mission_state == MissionState.TAKEOFF:
            # Check if all UAVs have reached takeoff altitude
            if await self._all_uavs_at_altitude():
                self.mission_state = MissionState.COVERAGE
                print("🔄 Takeoff complete, starting coverage mission")
                
        elif self.mission_state == MissionState.COVERAGE:
            # Simple coverage: move to formation positions
            if await self._coverage_complete():
                self.mission_state = MissionState.FORMATION
                print("🔄 Coverage complete, transitioning to formation")
                
        elif self.mission_state == MissionState.FORMATION:
            # Check if formation is stable
            if await self._formation_stable():
                self.mission_state = MissionState.LANDING
                print("🔄 Formation stable, starting landing sequence")
                
        elif self.mission_state == MissionState.LANDING:
            # Check if all UAVs have landed
            if await self._all_uavs_landed():
                self.mission_state = MissionState.COMPLETED
                print("🎉 Mission completed successfully!")
    
    async def _execute_mission_state(self):
        """Execute actions for the current mission state."""
        if self.mission_state == MissionState.INITIALIZING:
            await self._execute_initialization()
            
        elif self.mission_state == MissionState.TAKEOFF:
            await self._execute_takeoff()
            
        elif self.mission_state == MissionState.COVERAGE:
            await self._execute_coverage()
            
        elif self.mission_state == MissionState.FORMATION:
            await self._execute_formation()
            
        elif self.mission_state == MissionState.LANDING:
            await self._execute_landing()
    
    async def _execute_initialization(self):
        """Execute initialization sequence."""
        # Arm all UAVs
        print("🔒 Arming UAVs...")
        for uav_id, connector in self.uav_connectors.items():
            if connector.connected:
                await connector.arm()
    
    async def _execute_takeoff(self):
        """Execute takeoff sequence."""
        print("🚀 Executing takeoff...")
        for uav_id, connector in self.uav_connectors.items():
            if connector.connected and connector.armed:
                await connector.takeoff(self.takeoff_height)
    
    async def _execute_coverage(self):
        """Execute coverage mission."""
        # Move UAVs to formation positions
        for i, (uav_id, connector) in enumerate(self.uav_connectors.items()):
            if i < len(self.formation_positions):
                target_pos = self.formation_positions[i]
                await connector.set_position(*target_pos)
                
                # Create goal
                goal = UAVGoal(
                    uav_id=uav_id,
                    position=target_pos,
                    timestamp=time.time()
                )
                self.uav_goals[uav_id] = goal
    
    async def _execute_formation(self):
        """Execute formation flight."""
        # Maintain formation positions
        for uav_id, goal in self.uav_goals.items():
            if uav_id in self.uav_connectors:
                connector = self.uav_connectors[uav_id]
                await connector.set_position(*goal.position)
    
    async def _execute_landing(self):
        """Execute landing sequence."""
        print("🛬 Executing landing...")
        for uav_id, connector in self.uav_connectors.items():
            if connector.connected and connector.in_air:
                await connector.land()
    
    async def _all_uavs_ready(self) -> bool:
        """Check if all UAVs are ready for mission."""
        for uav_id, connector in self.uav_connectors.items():
            if not connector.connected or not connector.armed:
                return False
        return True
    
    async def _all_uavs_at_altitude(self) -> bool:
        """Check if all UAVs have reached takeoff altitude."""
        target_altitude = self.takeoff_height
        tolerance = self.position_tolerance
        
        for uav_id, connector in self.uav_connectors.items():
            if connector.connected:
                await connector.get_status()
                if not connector.in_air or abs(connector.position[2] - target_altitude) > tolerance:
                    return False
        return True
    
    async def _all_uavs_landed(self) -> bool:
        """Check if all UAVs have landed."""
        for uav_id, connector in self.uav_connectors.items():
            if connector.connected:
                await connector.get_status()
                if connector.in_air:
                    return False
        return True
    
    async def _coverage_complete(self) -> bool:
        """Check if coverage mission is complete."""
        # Simple check: all UAVs close to their formation positions
        for uav_id, goal in self.uav_goals.items():
            if uav_id in self.uav_connectors:
                connector = self.uav_connectors[uav_id]
                await connector.get_status()
                
                distance = self._calculate_distance(connector.position, goal.position)
                if distance > self.position_tolerance:
                    return False
        return True
    
    async def _formation_stable(self) -> bool:
        """Check if formation is stable."""
        # Check if all UAVs are close to their formation positions
        for uav_id, goal in self.uav_goals.items():
            if uav_id in self.uav_connectors:
                connector = self.uav_connectors[uav_id]
                await connector.get_status()
                
                distance = self._calculate_distance(connector.position, goal.position)
                if distance > self.position_tolerance:
                    return False
        return True
    
    async def _check_safety_conditions(self):
        """Check safety conditions."""
        # Check minimum separation between UAVs
        min_separation = 1.0
        
        uav_positions = []
        for uav_id, connector in self.uav_connectors.items():
            if connector.connected:
                await connector.get_status()
                uav_positions.append((uav_id, connector.position))
        
        # Check pairwise distances
        for i, (uav1_id, pos1) in enumerate(uav_positions):
            for j, (uav2_id, pos2) in enumerate(uav_positions[i+1:], i+1):
                distance = self._calculate_distance(pos1, pos2)
                if distance < min_separation:
                    print(f"⚠️  Safety warning: {uav1_id} and {uav2_id} too close ({distance:.2f}m)")
    
    async def _publish_status(self):
        """Publish status updates."""
        # Print status every few seconds
        if int(time.time()) % 5 == 0:
            print(f"\n📊 Status: {self.mission_state.value}")
            for uav_id, connector in self.uav_connectors.items():
                if connector.connected:
                    await connector.get_status()
                    pos = connector.position
                    print(f"   {uav_id}: pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}), "
                          f"armed={connector.armed}, in_air={connector.in_air}")
    
    def _calculate_distance(self, pos1: Tuple[float, float, float], 
                          pos2: Tuple[float, float, float]) -> float:
        """Calculate Euclidean distance between two positions."""
        return ((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2 + (pos1[2] - pos2[2])**2)**0.5
    
    async def cleanup(self):
        """Clean up resources."""
        print("🧹 Cleaning up...")
        
        # Stop mission if running
        if self.mission_state != MissionState.IDLE:
            await self.stop_mission()
        
        # Disconnect from UAVs
        for uav_id, connector in self.uav_connectors.items():
            await connector.disconnect()
        
        print("✅ Cleanup complete")


async def main():
    """Main function for testing the coordinator."""
    print("🧪 Test Coordinator for Placeholder UAVs")
    print("=" * 50)
    print("This coordinator will test basic UAV control logic")
    print("with placeholder UAVs running on ports 14540-14542.")
    print()
    
    # Create coordinator
    coordinator = TestCoordinator()
    
    try:
        # Initialize
        if not await coordinator.initialize():
            print("❌ Initialization failed")
            return
        
        # Start mission
        if not await coordinator.start_mission():
            print("❌ Failed to start mission")
            return
        
        # Run control loop
        await coordinator.run_control_loop()
        
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await coordinator.cleanup()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"❌ Coordinator failed: {e}")
        import traceback
        traceback.print_exc()
