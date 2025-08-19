#!/usr/bin/env python3
"""
Simple Control Test

This script tests basic control functionality with placeholder UAVs.
"""

import asyncio
import time
import json
import socket
from typing import Dict, Tuple


class SimpleControlTester:
    """Simple control tester for placeholder UAVs."""
    
    def __init__(self):
        self.ports = [14540, 14541, 14542]
        self.sockets: Dict[int, socket.socket] = {}
        
    async def setup(self):
        """Set up UDP sockets for testing."""
        print("🔌 Setting up test connections...")
        
        for port in self.ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.settimeout(1.0)
            self.sockets[port] = sock
        
        print("✅ Test connections ready")
    
    async def test_basic_commands(self):
        """Test basic UAV commands."""
        print("\n🧪 Testing basic UAV commands...")
        
        for port in self.ports:
            print(f"\n--- Testing port {port} ---")
            
            # Test status request
            await self._test_command(port, 'get_status')
            
            # Test arm command
            await self._test_command(port, 'arm')
            
            # Test takeoff command
            await self._test_command(port, 'takeoff', {'altitude': 3.0})
            
            # Test position command
            await self._test_command(port, 'set_position', {'x': 2.0, 'y': 1.0, 'z': 3.0})
            
            # Wait a bit
            await asyncio.sleep(2.0)
            
            # Test landing
            await self._test_command(port, 'land')
            
            # Test disarm
            await self._test_command(port, 'disarm')
    
    async def _test_command(self, port: int, command: str, params: dict = None):
        """Test a single command."""
        try:
            # Send command
            message = {
                'command': command,
                'uav_id': f'uav{port-14539}'
            }
            
            if params:
                message.update(params)
            
            data = json.dumps(message).encode('utf-8')
            self.sockets[port].sendto(data, ('127.0.0.1', port))
            
            print(f"   📤 Sent: {command}")
            
            # Wait for response
            try:
                response = await self._receive_response(port, timeout=2.0)
                if response:
                    print(f"   📥 Received: {response.get('command', 'unknown')}")
                    if 'success' in response:
                        success = response.get('success', False)
                        status = "✅" if success else "❌"
                        print(f"   {status} Success: {success}")
                else:
                    print(f"   ⏰ No response received")
                    
            except asyncio.TimeoutError:
                print(f"   ⏰ Response timeout")
                
        except Exception as e:
            print(f"   ❌ Error: {e}")
    
    async def _receive_response(self, port: int, timeout: float) -> dict:
        """Receive response from a specific port."""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            try:
                data, addr = self.sockets[port].recvfrom(1024)
                message = data.decode('utf-8')
                
                if message.startswith('{'):
                    return json.loads(message)
                    
            except socket.timeout:
                continue
            except Exception as e:
                print(f"   Error receiving response: {e}")
        
        return {}
    
    async def cleanup(self):
        """Clean up test resources."""
        print("\n🧹 Cleaning up test connections...")
        
        for sock in self.sockets.values():
            sock.close()
        
        self.sockets.clear()
        print("✅ Cleanup complete")


async def main():
    """Main test function."""
    print("🧪 Simple Control Test for Placeholder UAVs")
    print("=" * 50)
    print("This test will verify basic control functionality")
    print("with placeholder UAVs running on ports 14540-14542.")
    print()
    
    # Check if placeholder simulator is running
    print("⚠️  Make sure the placeholder UAV simulator is running first!")
    print("   Run: python3 scripts/placeholder_uav_simulator.py")
    print()
    
    input("Press Enter when the simulator is running...")
    
    # Create tester
    tester = SimpleControlTester()
    
    try:
        # Setup
        await tester.setup()
        
        # Run tests
        await tester.test_basic_commands()
        
        print("\n🎉 Basic control test completed!")
        
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    except Exception as e:
        print(f"\n❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
    finally:
        await tester.cleanup()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()
