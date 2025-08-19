#!/usr/bin/env python3
"""
Simple MAVSDK Connection Test

This script tests basic MAVSDK connectivity to the containers
without requiring PX4 SITL to be fully installed.
"""

import asyncio
import sys
import time
from typing import Dict, Optional

try:
    import mavsdk
    from mavsdk import System
    MAVSDK_AVAILABLE = True
except ImportError:
    MAVSDK_AVAILABLE = False
    print("❌ MAVSDK not available. Install with: pip install mavsdk")
    sys.exit(1)


class SimpleConnectionTester:
    """Simple connection tester for MAVSDK."""
    
    def __init__(self):
        self.connections: Dict[str, System] = {}
        self.connection_strings = {
            "uav1": "udpin://0.0.0.0:14540",
            "uav2": "udpin://0.0.0.0:14541", 
            "uav3": "udpin://0.0.0.0:14542"
        }
    
    async def test_connection(self, uav_id: str, connection_string: str) -> bool:
        """
        Test connection to a single UAV.
        
        Args:
            uav_id: UAV identifier
            connection_string: MAVSDK connection string
            
        Returns:
            True if connection successful, False otherwise
        """
        print(f"🔌 Testing connection to {uav_id} at {connection_string}")
        
        try:
            # Create drone system
            drone = System()
            
            # Try to connect
            print(f"   Connecting...")
            await drone.connect(system_address=connection_string)
            
            # Wait for connection with timeout
            connection_future = asyncio.create_task(self._wait_for_connection(drone))
            try:
                await asyncio.wait_for(connection_future, timeout=2.0)
                print(f"   ✅ {uav_id} connected successfully!")
                self.connections[uav_id] = drone
                return True
            except asyncio.TimeoutError:
                print(f"   ❌ {uav_id} connection timeout")
                await drone.disconnect()
                return False
                
        except Exception as e:
            print(f"   ❌ {uav_id} connection failed: {e}")
            return False
    
    async def _wait_for_connection(self, drone: System):
        """Wait for drone connection to be established."""
        async for state in drone.core.connection_state():
            if state.is_connected:
                return True
    
    async def test_all_connections(self) -> Dict[str, bool]:
        """
        Test connections to all UAVs.
        
        Returns:
            Dictionary mapping UAV IDs to connection success status
        """
        print("🚁 Testing MAVSDK connections to all UAVs...")
        print("=" * 50)
        
        results = {}
        
        for uav_id, connection_string in self.connection_strings.items():
            success = await self.test_connection(uav_id, connection_string)
            results[uav_id] = success
            print()  # Empty line for readability
        
        return results
    
    async def test_basic_communication(self, uav_id: str) -> bool:
        """
        Test basic communication with a connected UAV.
        
        Args:
            uav_id: UAV identifier
            
        Returns:
            True if communication successful, False otherwise
        """
        if uav_id not in self.connections:
            print(f"❌ {uav_id} not connected")
            return False
        
        drone = self.connections[uav_id]
        print(f"📡 Testing basic communication with {uav_id}")
        
        try:
            # Try to get system info
            print(f"   Getting system info...")
            system_info = await drone.info.get_version()
            print(f"   ✅ System info: {system_info.flight_sw_major}.{system_info.flight_sw_minor}")
            
            # Try to get connection state
            print(f"   Getting connection state...")
            async for state in drone.core.connection_state():
                print(f"   ✅ Connection state: {state}")
                break
            
            return True
            
        except Exception as e:
            print(f"   ❌ Communication failed: {e}")
            return False
    
    async def cleanup(self):
        """Clean up all connections."""
        print("🧹 Cleaning up connections...")
        for uav_id, drone in self.connections.items():
            try:
                await drone.disconnect()
                print(f"   ✅ Disconnected from {uav_id}")
            except Exception as e:
                print(f"   ❌ Error disconnecting from {uav_id}: {e}")
        
        self.connections.clear()


async def main():
    """Main test function."""
    print("🧪 Simple MAVSDK Connection Test")
    print("=" * 50)
    print("This test will attempt to connect to the containers")
    print("and verify basic MAVSDK functionality.")
    print()
    
    # Check if containers are running
    import subprocess
    try:
        result = subprocess.run(['docker', 'ps', '--filter', 'name=px4_sitl', '--format', '{{.Names}}'], 
                              capture_output=True, text=True, check=True)
        running_containers = result.stdout.strip().split('\n') if result.stdout.strip() else []
        
        if not running_containers:
            print("❌ No PX4 SITL containers are running!")
            print("   Start them with: docker-compose -f docker-compose-px4.yml up -d")
            return
        
        print(f"✅ Found running containers: {', '.join(running_containers)}")
        print()
        
    except subprocess.CalledProcessError:
        print("❌ Could not check container status")
        return
    
    # Create tester
    tester = SimpleConnectionTester()
    
    try:
        # Test connections
        results = await tester.test_all_connections()
        
        # Summary
        print("=" * 50)
        print("📊 Connection Test Results:")
        for uav_id, success in results.items():
            status = "✅ SUCCESS" if success else "❌ FAILED"
            print(f"   {uav_id}: {status}")
        
        # Test communication for successful connections
        if any(results.values()):
            print()
            print("📡 Testing basic communication...")
            for uav_id, success in results.items():
                if success:
                    await tester.test_basic_communication(uav_id)
                    print()
        
        # Recommendations
        print("=" * 50)
        print("💡 Recommendations:")
        
        successful_connections = sum(results.values())
        if successful_connections == 0:
            print("   ❌ No connections successful. Check:")
            print("      - Container status and logs")
            print("      - Port availability (14540-14542)")
            print("      - Network configuration")
        elif successful_connections < len(results):
            print("   ⚠️  Partial connectivity. Some UAVs may need:")
            print("      - Container restart")
            print("      - Port conflict resolution")
        else:
            print("   ✅ All connections successful!")
            print("   🚀 Ready to implement UAV control logic")
        
        print()
        print("🔧 Next steps:")
        print("   1. If connections fail: Check container logs")
        print("   2. If connections succeed: Test basic control commands")
        print("   3. Implement coordinator control loop")
        
    finally:
        await tester.cleanup()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
