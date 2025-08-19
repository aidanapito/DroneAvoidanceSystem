#!/usr/bin/env python3
"""
Run Gazebo Formation Flying Simulation

This script sets up and runs the formation flying simulation in Gazebo.
"""

import subprocess
import sys
import os
import time
from pathlib import Path


def check_dependencies():
    """Check if required dependencies are installed."""
    print("🔍 Checking dependencies...")
    
    # Check if ROS 2 is available
    try:
        result = subprocess.run(['ros2', '--version'], capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ ROS 2 found")
        else:
            print("❌ ROS 2 not found")
            return False
    except FileNotFoundError:
        print("❌ ROS 2 not found in PATH")
        return False
    
    # Check if Gazebo is available
    try:
        result = subprocess.run(['gazebo', '--version'], capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ Gazebo found")
        else:
            print("❌ Gazebo not found")
            return False
    except FileNotFoundError:
        print("❌ Gazebo not found in PATH")
        return False
    
    return True


def build_workspace():
    """Build the ROS 2 workspace."""
    print("\n🔨 Building workspace...")
    
    workspace_path = Path(__file__).parent.parent
    os.chdir(workspace_path)
    
    try:
        # Source ROS 2
        if 'ROS_DISTRO' not in os.environ:
            print("📦 Sourcing ROS 2...")
            subprocess.run(['source', '/opt/ros/humble/setup.bash'], shell=True, check=True)
        
        # Build the workspace
        print("🔨 Building with colcon...")
        result = subprocess.run(['colcon', 'build', '--packages-select', 'swarm_core'], 
                              capture_output=True, text=True, check=True)
        
        if result.returncode == 0:
            print("✅ Workspace built successfully")
            return True
        else:
            print("❌ Build failed")
            print(result.stderr)
            return False
            
    except subprocess.CalledProcessError as e:
        print(f"❌ Build error: {e}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False


def run_formation_simulation():
    """Run the formation flying simulation."""
    print("\n🚁 Starting Formation Flying Simulation...")
    
    workspace_path = Path(__file__).parent.parent
    os.chdir(workspace_path)
    
    try:
        # Source the workspace
        print("📦 Sourcing workspace...")
        setup_script = workspace_path / "install" / "setup.bash"
        if not setup_script.exists():
            print("❌ Workspace not built. Run build first.")
            return False
        
        # Launch the formation simulation
        print("🚀 Launching formation simulation...")
        launch_file = "src/swarm_core/launch/formation_flying.launch.py"
        
        cmd = [
            'ros2', 'launch', 'swarm_core', 'formation_flying.launch.py',
            'use_gazebo:=true',
            'use_rviz:=true',
            'drone_count:=3'
        ]
        
        print(f"Running: {' '.join(cmd)}")
        print("\n📋 Simulation will start:")
        print("   - Gazebo with 3 drones")
        print("   - Formation controller")
        print("   - RViz visualization")
        print("   - Automatic formation changes every 15 seconds")
        print("\n🎮 Controls:")
        print("   - Watch drones fly in formations")
        print("   - Press Ctrl+C to stop")
        print("   - Use RViz to see formation markers")
        
        # Run the launch file
        result = subprocess.run(cmd, check=True)
        
        return True
        
    except subprocess.CalledProcessError as e:
        print(f"❌ Launch error: {e}")
        return False
    except KeyboardInterrupt:
        print("\n⏹️  Simulation stopped by user")
        return True
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        return False


def main():
    """Main function."""
    print("🚁 Gazebo Formation Flying Simulation")
    print("=" * 50)
    
    # Check dependencies
    if not check_dependencies():
        print("\n❌ Missing dependencies. Please install:")
        print("   - ROS 2 Humble")
        print("   - Gazebo")
        print("   - Required ROS 2 packages")
        return False
    
    # Build workspace
    if not build_workspace():
        print("\n❌ Failed to build workspace")
        return False
    
    # Run simulation
    if not run_formation_simulation():
        print("\n❌ Failed to run simulation")
        return False
    
    print("\n🎉 Formation flying simulation completed!")
    return True


if __name__ == "__main__":
    try:
        success = main()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n⏹️  Script interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Script failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
