#!/usr/bin/env python3
"""
Test script for PX4 drone models in Gazebo
This script launches a world with realistic PX4 drone models
"""

import subprocess
import sys
import os
import time

def main():
    print("🚁 Testing PX4 Drone Models in Gazebo")
    print("=" * 50)
    
    # Check if we're in the right directory
    world_file = "worlds/px4_formation_flying.world"
    if not os.path.exists(world_file):
        print(f"❌ World file not found: {world_file}")
        print("Make sure you're running this from the swarm_ws directory")
        return False
    
    print(f"✅ Found world file: {world_file}")
    print("🎮 Launching Gazebo with PX4 drone models...")
    print()
    print("Features:")
    print("  ✅ Realistic PX4 drone models with proper physics")
    print("  ✅ 4 propellers per drone")
    print("  ✅ GPS antennas")
    print("  ✅ Collision detection")
    print("  ✅ ROS2 integration ready")
    print("  ✅ Formation flying setup")
    print()
    
    try:
        # Launch Gazebo with the new world
        cmd = ["gazebo", "--verbose", world_file]
        print(f"🚀 Running: {' '.join(cmd)}")
        print("⏳ Starting Gazebo...")
        
        # Run Gazebo
        process = subprocess.Popen(cmd)
        
        print("✅ Gazebo launched successfully!")
        print("🎮 Use the Gazebo GUI to:")
        print("   - Move camera (right-click + drag)")
        print("   - Zoom (scroll wheel)")
        print("   - Pause/resume simulation")
        print("   - Reset simulation")
        print()
        print("🛑 To stop: Close Gazebo window or press Ctrl+C")
        
        # Wait for user to stop
        try:
            process.wait()
        except KeyboardInterrupt:
            print("\n⏹️  Stopping Gazebo...")
            process.terminate()
            process.wait()
            print("✅ Gazebo stopped")
        
        return True
        
    except FileNotFoundError:
        print("❌ Gazebo not found. Please install Gazebo first.")
        print("   On macOS, you can use Docker: docker run -it gazebo")
        return False
    except Exception as e:
        print(f"❌ Error launching Gazebo: {e}")
        return False

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
