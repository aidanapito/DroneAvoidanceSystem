#!/usr/bin/env python3
"""
Placeholder UAV Simulator

This script creates a simple UAV simulator that responds to MAVSDK commands
for testing the control loop without requiring full PX4 SITL installation.
"""

import asyncio
import time
import math
from typing import Dict, Optional, Tuple
from dataclasses import dataclass
import socket
import threading
import json

# MAVLink message definitions (simplified)
MAV_CMD_ARM = 400
MAV_CMD_DISARM = 401
MAV_CMD_NAV_TAKEOFF = 22
MAV_CMD_NAV_LAND = 23
MAV_CMD_DO_SET_MODE = 176

# Flight modes
FLIGHT_MODE_MANUAL = 0
FLIGHT_MODE_OFFBOARD = 4
FLIGHT_MODE_AUTO = 3


@dataclass
class UAVState:
    """UAV state information."""
    armed: bool = False
    in_air: bool = False
    flight_mode: int = FLIGHT_MODE_MANUAL
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    attitude: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # roll, pitch, yaw
    battery_percentage: float = 100.0
    connected: bool = False


class PlaceholderUAV:
    """Placeholder UAV simulator."""
    
    def __init__(self, uav_id: str, port: int):
        self.uav_id = uav_id
        self.port = port
        self.state = UAVState()
        self.socket = None
        self.running = False
        self.control_thread = None
        
        # Control parameters
        self.max_velocity = 5.0  # m/s
        self.max_acceleration = 3.0  # m/s²
        self.control_rate = 50.0  # Hz
        
        # Target position for offboard control
        self.target_position = (0.0, 0.0, 0.0)
        self.target_velocity = (0.0, 0.0, 0.0)
        
        # PID control gains (simplified)
        self.kp_pos = 2.0
        self.kd_vel = 1.0
        
    async def start(self):
        """Start the UAV simulator."""
        print(f"🚁 Starting {self.uav_id} simulator on port {self.port}")
        
        # Create UDP socket
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind(('localhost', self.port))
        self.socket.settimeout(0.1)
        
        self.state.connected = True
        self.running = True
        
        # Start control loop
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        
        # Start message handling
        asyncio.create_task(self._handle_messages())
        
        print(f"✅ {self.uav_id} simulator started")
    
    async def stop(self):
        """Stop the UAV simulator."""
        print(f"🛑 Stopping {self.uav_id} simulator")
        
        self.running = False
        if self.socket:
            self.socket.close()
        
        if self.control_thread and self.control_thread.is_alive():
            self.control_thread.join(timeout=2.0)
        
        self.state.connected = False
        print(f"✅ {self.uav_id} simulator stopped")
    
    async def _handle_messages(self):
        """Handle incoming MAVLink-like messages."""
        while self.running:
            try:
                # Receive message
                data, addr = self.socket.recvfrom(1024)
                message = data.decode('utf-8')
                
                # Parse and handle message
                await self._process_message(message)
                
            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    print(f"Error handling message for {self.uav_id}: {e}")
    
    async def _process_message(self, message: str):
        """Process a single message."""
        try:
            # Simple JSON-based message format for testing
            if message.startswith('{'):
                data = json.loads(message)
                command = data.get('command')
                
                if command == 'arm':
                    await self._arm()
                elif command == 'disarm':
                    await self._disarm()
                elif command == 'takeoff':
                    altitude = data.get('altitude', 5.0)
                    await self._takeoff(altitude)
                elif command == 'land':
                    await self._land()
                elif command == 'set_mode':
                    mode = data.get('mode', FLIGHT_MODE_MANUAL)
                    await self._set_flight_mode(mode)
                elif command == 'set_position':
                    x = data.get('x', 0.0)
                    y = data.get('y', 0.0)
                    z = data.get('z', 0.0)
                    self.target_position = (x, y, z)
                elif command == 'set_velocity':
                    vx = data.get('vx', 0.0)
                    vy = data.get('vy', 0.0)
                    vz = data.get('vz', 0.0)
                    self.target_velocity = (vx, vy, vz)
                elif command == 'get_status':
                    await self._send_status()
                    
        except Exception as e:
            print(f"Error processing message for {self.uav_id}: {e}")
    
    async def _arm(self):
        """Arm the UAV."""
        if not self.state.armed:
            self.state.armed = True
            print(f"🔒 {self.uav_id} armed")
            await self._send_response('arm', {'success': True})
    
    async def _disarm(self):
        """Disarm the UAV."""
        if self.state.armed:
            self.state.armed = False
            self.state.in_air = False
            print(f"🔓 {self.uav_id} disarmed")
            await self._send_response('disarm', {'success': True})
    
    async def _takeoff(self, altitude: float):
        """Take off to specified altitude."""
        if not self.state.armed:
            await self._send_response('takeoff', {'success': False, 'error': 'Not armed'})
            return
        
        print(f"🚀 {self.uav_id} taking off to {altitude}m")
        self.state.in_air = True
        self.target_position = (0.0, 0.0, altitude)
        await self._send_response('takeoff', {'success': True})
    
    async def _land(self):
        """Land the UAV."""
        if not self.state.in_air:
            await self._send_response('land', {'success': False, 'error': 'Not in air'})
            return
        
        print(f"🛬 {self.uav_id} landing")
        self.target_position = (0.0, 0.0, 0.0)
        await self._send_response('land', {'success': True})
    
    async def _set_flight_mode(self, mode: int):
        """Set flight mode."""
        self.state.flight_mode = mode
        mode_name = {FLIGHT_MODE_MANUAL: 'MANUAL', 
                    FLIGHT_MODE_OFFBOARD: 'OFFBOARD',
                    FLIGHT_MODE_AUTO: 'AUTO'}.get(mode, 'UNKNOWN')
        
        print(f"🔄 {self.uav_id} flight mode: {mode_name}")
        await self._send_response('set_mode', {'success': True, 'mode': mode})
    
    async def _send_status(self):
        """Send current UAV status."""
        status = {
            'command': 'status_response',
            'uav_id': self.uav_id,
            'armed': self.state.armed,
            'in_air': self.state.in_air,
            'flight_mode': self.state.flight_mode,
            'position': self.state.position,
            'velocity': self.state.velocity,
            'attitude': self.state.attitude,
            'battery_percentage': self.state.battery_percentage,
            'connected': self.state.connected
        }
        
        await self._send_response('status', status)
    
    async def _send_response(self, command: str, data: dict):
        """Send response message."""
        if not self.socket:
            return
        
        response = {
            'command': f'{command}_response',
            'uav_id': self.uav_id,
            **data
        }
        
        try:
            message = json.dumps(response).encode('utf-8')
            self.socket.sendto(message, ('localhost', self.port))
        except Exception as e:
            print(f"Error sending response for {self.uav_id}: {e}")
    
    def _control_loop(self):
        """Main control loop running in separate thread."""
        last_time = time.time()
        
        while self.running:
            current_time = time.time()
            dt = current_time - last_time
            
            if dt >= 1.0 / self.control_rate:
                self._update_state(dt)
                last_time = current_time
            
            time.sleep(0.001)  # Small sleep to prevent busy waiting
    
    def _update_state(self, dt: float):
        """Update UAV state based on control inputs."""
        if not self.state.in_air:
            return
        
        # Simple position control (PID-like)
        current_pos = self.state.position
        target_pos = self.target_position
        
        # Position error
        pos_error = (
            target_pos[0] - current_pos[0],
            target_pos[1] - current_pos[1],
            target_pos[2] - current_pos[2]
        )
        
        # Velocity command (P control)
        vel_cmd = (
            self.kp_pos * pos_error[0],
            self.kp_pos * pos_error[1],
            self.kp_pos * pos_error[2]
        )
        
        # Apply velocity limits
        vel_cmd = self._limit_velocity(vel_cmd)
        
        # Update velocity
        self.state.velocity = vel_cmd
        
        # Update position (simple integration)
        new_pos = (
            current_pos[0] + vel_cmd[0] * dt,
            current_pos[1] + vel_cmd[1] * dt,
            current_pos[2] + vel_cmd[2] * dt
        )
        
        self.state.position = new_pos
        
        # Update attitude (simple heading control)
        if abs(vel_cmd[0]) > 0.1 or abs(vel_cmd[1]) > 0.1:
            yaw = math.atan2(vel_cmd[1], vel_cmd[0])
            self.state.attitude = (0.0, 0.0, yaw)
        
        # Simulate battery drain
        if self.state.in_air:
            self.state.battery_percentage -= 0.01  # Very slow drain
    
    def _limit_velocity(self, velocity: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Limit velocity to safe values."""
        vx, vy, vz = velocity
        magnitude = math.sqrt(vx**2 + vy**2 + vz**2)
        
        if magnitude > self.max_velocity:
            scale = self.max_velocity / magnitude
            return (vx * scale, vy * scale, vz * scale)
        
        return velocity


class PlaceholderSwarmSimulator:
    """Manages multiple placeholder UAV simulators."""
    
    def __init__(self):
        self.uavs: Dict[str, PlaceholderUAV] = {}
        self.ports = [14540, 14541, 14542]
    
    async def start_swarm(self):
        """Start all UAV simulators."""
        print("🚁 Starting placeholder UAV swarm simulator...")
        
        for i, port in enumerate(self.ports):
            uav_id = f"uav{i+1}"
            uav = PlaceholderUAV(uav_id, port)
            await uav.start()
            self.uavs[uav_id] = uav
        
        print(f"✅ Started {len(self.uavs)} UAV simulators")
    
    async def stop_swarm(self):
        """Stop all UAV simulators."""
        print("🛑 Stopping placeholder UAV swarm simulator...")
        
        for uav in self.uavs.values():
            await uav.stop()
        
        self.uavs.clear()
        print("✅ Stopped all UAV simulators")
    
    def get_uav_status(self, uav_id: str) -> Optional[UAVState]:
        """Get status of a specific UAV."""
        if uav_id in self.uavs:
            return self.uavs[uav_id].state
        return None
    
    def get_swarm_status(self) -> Dict[str, UAVState]:
        """Get status of all UAVs."""
        return {uav_id: uav.state for uav_id, uav in self.uavs.items()}


async def main():
    """Main function for testing the placeholder simulator."""
    print("🧪 Placeholder UAV Swarm Simulator")
    print("=" * 50)
    print("This simulator creates placeholder UAVs that respond to")
    print("MAVSDK-like commands for testing the control loop.")
    print()
    
    simulator = PlaceholderSwarmSimulator()
    
    try:
        # Start swarm
        await simulator.start_swarm()
        
        print()
        print("🚀 Simulator running! You can now:")
        print("   1. Test MAVSDK connections to ports 14540-14542")
        print("   2. Send commands via UDP to test control logic")
        print("   3. Run the coordinator to test the full system")
        print()
        print("📱 Example commands (send via UDP to localhost:port):")
        print("   {\"command\": \"arm\"}")
        print("   {\"command\": \"takeoff\", \"altitude\": 5.0}")
        print("   {\"command\": \"set_position\", \"x\": 2.0, \"y\": 1.0, \"z\": 5.0}")
        print("   {\"command\": \"get_status\"}")
        print()
        print("⏹️  Press Ctrl+C to stop...")
        
        # Keep running
        while True:
            await asyncio.sleep(1.0)
            
            # Print status every 10 seconds
            if int(time.time()) % 10 == 0:
                print("\n📊 Current Status:")
                for uav_id, uav in simulator.uavs.items():
                    pos = uav.state.position
                    print(f"   {uav_id}: pos=({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}), "
                          f"armed={uav.state.armed}, in_air={uav.state.in_air}")
    
    except KeyboardInterrupt:
        print("\n⏹️  Stopping simulator...")
    finally:
        await simulator.stop_swarm()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"❌ Simulator failed: {e}")
        import traceback
        traceback.print_exc()
