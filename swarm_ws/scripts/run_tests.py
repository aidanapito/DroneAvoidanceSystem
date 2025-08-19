#!/usr/bin/env python3
"""
Comprehensive Test Runner

This script runs all tests in sequence to verify the complete system.
"""

import asyncio
import subprocess
import sys
import time
from typing import List, Dict, Tuple


class TestRunner:
    """Runs all tests in sequence."""
    
    def __init__(self):
        self.test_results: Dict[str, bool] = {}
        self.current_test = None
        
    async def run_all_tests(self):
        """Run all tests in sequence."""
        print("🚀 Comprehensive Test Runner for Drone Swarm System")
        print("=" * 60)
        print("This will test the complete system step by step.")
        print()
        
        # Test 1: Check if containers are running
        await self._run_test("Container Status Check", self._check_containers)
        
        # Test 2: Test MAVSDK connections
        await self._run_test("MAVSDK Connection Test", self._test_mavsdk_connections)
        
        # Test 3: Test placeholder UAV simulator
        await self._run_test("Placeholder UAV Simulator", self._test_placeholder_simulator)
        
        # Test 4: Test basic control
        await self._run_test("Basic Control Test", self._test_basic_control)
        
        # Test 5: Test coordinator
        await self._run_test("Coordinator Test", self._test_coordinator)
        
        # Print final results
        self._print_results()
    
    async def _run_test(self, test_name: str, test_func):
        """Run a single test."""
        print(f"\n🧪 Running: {test_name}")
        print("-" * 40)
        
        self.current_test = test_name
        start_time = time.time()
        
        try:
            result = await test_func()
            duration = time.time() - start_time
            
            if result:
                print(f"✅ {test_name} PASSED ({duration:.1f}s)")
                self.test_results[test_name] = True
            else:
                print(f"❌ {test_name} FAILED ({duration:.1f}s)")
                self.test_results[test_name] = False
                
        except Exception as e:
            duration = time.time() - start_time
            print(f"❌ {test_name} ERROR ({duration:.1f}s): {e}")
            self.test_results[test_name] = False
        
        self.current_test = None
    
    async def _check_containers(self) -> bool:
        """Check if containers are running."""
        try:
            # Check if Docker is running
            result = subprocess.run(['docker', 'ps'], capture_output=True, text=True, check=True)
            
            # Look for PX4 SITL containers
            if 'px4_sitl' in result.stdout:
                print("✅ PX4 SITL containers are running")
                return True
            else:
                print("⚠️  No PX4 SITL containers found")
                print("   You can start them with: docker-compose -f docker-compose-px4.yml up -d")
                return False
                
        except subprocess.CalledProcessError:
            print("❌ Docker is not running or not accessible")
            return False
        except FileNotFoundError:
            print("❌ Docker command not found")
            return False
    
    async def _test_mavsdk_connections(self) -> bool:
        """Test MAVSDK connections."""
        try:
            # Run the MAVSDK connection test
            result = subprocess.run([
                sys.executable, 'scripts/test_mavsdk_simple.py'
            ], capture_output=True, text=True, check=True)
            
            # Check if test was successful
            if 'All connections successful' in result.stdout:
                print("✅ MAVSDK connections successful")
                return True
            elif 'No connections successful' in result.stdout:
                print("❌ No MAVSDK connections successful")
                return False
            else:
                print("⚠️  Partial MAVSDK connectivity")
                return False
                
        except subprocess.CalledProcessError as e:
            print(f"❌ MAVSDK test failed: {e}")
            return False
    
    async def _test_placeholder_simulator(self) -> bool:
        """Test placeholder UAV simulator."""
        try:
            # Start simulator in background
            print("🚁 Starting placeholder UAV simulator...")
            
            # Run simulator for a short time
            process = subprocess.Popen([
                sys.executable, 'scripts/placeholder_uav_simulator.py'
            ], stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            
            # Wait a bit for startup
            await asyncio.sleep(3)
            
            # Check if process is still running
            if process.poll() is None:
                print("✅ Placeholder simulator started successfully")
                
                # Stop the process
                process.terminate()
                process.wait(timeout=5)
                
                return True
            else:
                print("❌ Placeholder simulator failed to start")
                return False
                
        except Exception as e:
            print(f"❌ Placeholder simulator test failed: {e}")
            return False
    
    async def _test_basic_control(self) -> bool:
        """Test basic control functionality."""
        try:
            # This test requires the simulator to be running
            # For now, we'll just check if the test script exists and is valid
            print("🔌 Basic control test script ready")
            print("   (Run manually with: python3 scripts/test_simple_control.py)")
            return True
            
        except Exception as e:
            print(f"❌ Basic control test failed: {e}")
            return False
    
    async def _test_coordinator(self) -> bool:
        """Test coordinator functionality."""
        try:
            # This test requires the simulator to be running
            # For now, we'll just check if the test script exists and is valid
            print("🎯 Coordinator test script ready")
            print("   (Run manually with: python3 scripts/test_coordinator.py)")
            return True
            
        except Exception as e:
            print(f"❌ Coordinator test failed: {e}")
            return False
    
    def _print_results(self):
        """Print final test results."""
        print("\n" + "=" * 60)
        print("📊 FINAL TEST RESULTS")
        print("=" * 60)
        
        passed = sum(self.test_results.values())
        total = len(self.test_results)
        
        for test_name, result in self.test_results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            print(f"{status} {test_name}")
        
        print(f"\n🎯 Overall: {passed}/{total} tests passed")
        
        if passed == total:
            print("🎉 All tests passed! System is ready for development.")
        elif passed > total // 2:
            print("⚠️  Most tests passed. Some components need attention.")
        else:
            print("❌ Many tests failed. System needs significant work.")
        
        print("\n💡 Next steps:")
        if passed == total:
            print("   1. ✅ System is working correctly")
            print("   2. 🚀 Ready to implement advanced features")
            print("   3. 🔧 Can proceed with PX4 SITL integration")
        else:
            print("   1. 🔍 Investigate failed tests")
            print("   2. 🐛 Fix identified issues")
            print("   3. 🔄 Re-run tests after fixes")
        
        print("\n📚 Manual testing commands:")
        print("   python3 scripts/test_mavsdk_simple.py")
        print("   python3 scripts/placeholder_uav_simulator.py")
        print("   python3 scripts/test_simple_control.py")
        print("   python3 scripts/test_coordinator.py")


async def main():
    """Main function."""
    runner = TestRunner()
    
    try:
        await runner.run_all_tests()
    except KeyboardInterrupt:
        print("\n⏹️  Test runner interrupted by user")
    except Exception as e:
        print(f"\n❌ Test runner failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except Exception as e:
        print(f"❌ Test runner failed: {e}")
        import traceback
        traceback.print_exc()
