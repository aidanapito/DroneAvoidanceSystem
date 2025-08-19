#!/bin/bash
# macOS Docker-based Setup for ROS 2 and Gazebo Formation Flying

set -e

echo "🍎 Setting up macOS environment for ROS 2 and Gazebo Formation Flying using Docker..."
echo "=" * 70

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "❌ Docker not found. Installing Docker Desktop first..."
    echo "📥 Please download and install Docker Desktop from:"
    echo "   https://www.docker.com/products/docker-desktop"
    echo ""
    echo "After installation, restart your terminal and run this script again."
    exit 1
else
    echo "✅ Docker found: $(docker --version)"
fi

# Check if Docker is running
if ! docker info &> /dev/null; then
    echo "❌ Docker is not running. Please start Docker Desktop and try again."
    exit 1
else
    echo "✅ Docker is running"
fi

# Install Python dependencies
echo "🐍 Installing Python dependencies..."
pip3 install numpy matplotlib

# Create Docker Compose file for ROS 2 + Gazebo
echo "📝 Creating Docker Compose configuration..."
cat > docker-compose-ros2-gazebo.yml << 'EOF'
version: '3.8'

services:
  # ROS 2 + Gazebo container
  ros2_gazebo:
    image: osrf/ros:humble-desktop-full
    container_name: ros2_gazebo_formation
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=0
      - GAZEBO_MODEL_PATH=/opt/ros/humble/share/gazebo-11/models
    volumes:
      - .:/workspace
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ~/.Xauthority:/root/.Xauthority:rw
    ports:
      - "11311:11311"  # ROS master
      - "14540:14540"  # MAVSDK port 1
      - "14541:14541"  # MAVSDK port 2
      - "14542:14542"  # MAVSDK port 3
    network_mode: host
    privileged: true
    stdin_open: true
    tty: true
    command: >
      bash -c "
        echo '🚀 ROS 2 + Gazebo container starting...' &&
        echo 'Installing additional packages...' &&
        apt-get update &&
        apt-get install -y python3-pip python3-numpy python3-matplotlib &&
        pip3 install mavsdk &&
        echo 'Starting Gazebo...' &&
        gazebo --verbose /opt/ros/humble/share/gazebo-11/worlds/empty.world &
        echo 'Starting ROS 2...' &&
        source /opt/ros/humble/setup.bash &&
        echo 'Container ready! Use docker exec to run commands.' &&
        tail -f /dev/null
      "

  # Formation controller container
  formation_controller:
    image: osrf/ros:humble-desktop-full
    container_name: formation_controller
    environment:
      - ROS_DOMAIN_ID=0
    volumes:
      - .:/workspace
    depends_on:
      - ros2_gazebo
    command: >
      bash -c "
        echo '🤖 Formation controller starting...' &&
        source /opt/ros/humble/setup.bash &&
        cd /workspace &&
        echo 'Building workspace...' &&
        colcon build --packages-select swarm_core &&
        echo 'Formation controller ready!' &&
        tail -f /dev/null
      "
EOF

echo "✅ Docker Compose file created"

# Create workspace setup script
echo "📝 Creating workspace setup script..."
cat > setup_docker_workspace.sh << 'EOF'
#!/bin/bash
# Docker workspace setup script

echo "🚀 Setting up Docker ROS 2 workspace environment..."

# Check if containers are running
if ! docker ps | grep -q ros2_gazebo_formation; then
    echo "❌ ROS 2 + Gazebo container not running. Start with:"
    echo "   docker-compose -f docker-compose-ros2-gazebo.yml up -d"
    exit 1
fi

# Set environment variables
export ROS_DOMAIN_ID=0
export DISPLAY=:0

echo "✅ Docker workspace environment ready!"
echo "   ROS 2: Available in ros2_gazebo_formation container"
echo "   Gazebo: Available in ros2_gazebo_formation container"
echo "   Formation Controller: Available in formation_controller container"
echo ""
echo "🚁 Ready for formation flying simulation!"
echo ""
echo "📋 Next steps:"
echo "1. Start containers: docker-compose -f docker-compose-ros2-gazebo.yml up -d"
echo "2. Access ROS 2: docker exec -it ros2_gazebo_formation bash"
echo "3. Access controller: docker exec -it formation_controller bash"
echo "4. Build workspace: colcon build --packages-select swarm_core"
echo "5. Run formation flying: ros2 launch swarm_core formation_flying.launch.py"
EOF

chmod +x setup_docker_workspace.sh

# Create quick start script
echo "📝 Creating quick start script..."
cat > run_formation_docker.sh << 'EOF'
#!/bin/bash
# Quick start script for Docker-based formation flying

echo "🚁 Starting Formation Flying Simulation with Docker..."
echo "=" * 50

# Check if Docker is running
if ! docker info &> /dev/null; then
    echo "❌ Docker is not running. Please start Docker Desktop."
    exit 1
fi

# Start containers
echo "🐳 Starting ROS 2 + Gazebo containers..."
docker-compose -f docker-compose-ros2-gazebo.yml up -d

# Wait for containers to be ready
echo "⏳ Waiting for containers to be ready..."
sleep 10

# Check container status
echo "📊 Container status:"
docker-compose -f docker-compose-ros2-gazebo.yml ps

echo ""
echo "🎯 Containers are running!"
echo ""
echo "📋 To access the simulation:"
echo "1. Open Gazebo: docker exec -it ros2_gazebo_formation gazebo"
echo "2. Access ROS 2: docker exec -it ros2_gazebo_formation bash"
echo "3. Run formation flying:"
echo "   docker exec -it formation_controller bash"
echo "   source /opt/ros/humble/setup.bash"
echo "   cd /workspace"
echo "   colcon build --packages-select swarm_core"
echo "   ros2 launch swarm_core formation_flying.launch.py"
echo ""
echo "🛑 To stop: docker-compose -f docker-compose-ros2-gazebo.yml down"
EOF

chmod +x run_formation_docker.sh

# Create simplified formation test
echo "📝 Creating simplified formation test..."
cat > test_formation_simple.py << 'EOF'
#!/usr/bin/env python3
"""
Simple Formation Flying Test (No ROS 2 required)

This script demonstrates formation flying logic with visualization
that can run on macOS without ROS 2 or Gazebo.
"""

import numpy as np
import matplotlib.pyplot as plt
import time
from typing import List, Tuple

class SimpleFormationTest:
    """Simple formation flying test with matplotlib visualization."""
    
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('Simple Formation Flying Test')
        
        # Drone positions
        self.drone_positions = [
            np.array([0.0, 0.0]),    # Center
            np.array([-3.0, -3.0]),  # Left back
            np.array([3.0, -3.0])    # Right back
        ]
        
        # Formation patterns
        self.formations = {
            'triangle': [
                (0.0, 0.0), (-3.0, -3.0), (3.0, -3.0)
            ],
            'line': [
                (-3.0, 0.0), (0.0, 0.0), (3.0, 0.0)
            ],
            'square': [
                (-1.5, -1.5), (1.5, -1.5), (0.0, 1.5)
            ],
            'v_formation': [
                (0.0, 0.0), (-3.0, -3.0), (3.0, -3.0)
            ]
        }
        
        self.current_formation = 'triangle'
        self.formation_index = 0
        
    def update_formation(self):
        """Change to next formation pattern."""
        formations = list(self.formations.keys())
        self.formation_index = (self.formation_index + 1) % len(formations)
        self.current_formation = formations[self.formation_index]
        
        # Update target positions
        targets = self.formations[self.current_formation]
        for i, (x, y) in enumerate(targets):
            if i < len(self.drone_positions):
                self.drone_positions[i] = np.array([x, y])
        
        print(f"🔄 Changed to {self.current_formation} formation")
    
    def animate(self):
        """Animate the formation flying."""
        print("🚁 Starting Simple Formation Flying Test")
        print("=" * 50)
        print("This demonstrates formation logic without ROS 2")
        print("Press Ctrl+C to stop")
        print()
        
        start_time = time.time()
        last_change = start_time
        
        try:
            while True:
                current_time = time.time()
                
                # Change formation every 5 seconds
                if current_time - last_change > 5.0:
                    self.update_formation()
                    last_change = current_time
                
                # Clear plot
                self.ax.clear()
                self.ax.set_xlim(-10, 10)
                self.ax.set_ylim(-10, 10)
                self.ax.set_aspect('equal')
                self.ax.grid(True, alpha=0.3)
                self.ax.set_title(f'Formation: {self.current_formation.upper()}')
                
                # Plot drones
                colors = ['red', 'blue', 'green']
                for i, pos in enumerate(self.drone_positions):
                    self.ax.plot(pos[0], pos[1], 'o', color=colors[i], 
                               markersize=15, label=f'Drone {i+1}')
                
                # Plot formation center
                center = np.mean(self.drone_positions, axis=0)
                self.ax.plot(center[0], center[1], 's', color='yellow', 
                           markersize=20, label='Formation Center')
                
                # Add legend and info
                self.ax.legend()
                self.ax.text(0.02, 0.98, f'Time: {current_time - start_time:.1f}s', 
                           transform=self.ax.transAxes, verticalalignment='top',
                           bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
                
                # Update display
                plt.draw()
                plt.pause(0.1)
                
        except KeyboardInterrupt:
            print("\n⏹️  Test stopped by user")
            print("🎉 Simple formation test completed!")

if __name__ == "__main__":
    test = SimpleFormationTest()
    test.animate()
EOF

chmod +x test_formation_simple.py

echo ""
echo "🎉 Docker-based setup completed successfully!"
echo "=" * 70
echo ""
echo "📋 What was created:"
echo "   ✅ docker-compose-ros2-gazebo.yml - ROS 2 + Gazebo containers"
echo "   ✅ setup_docker_workspace.sh - Workspace setup script"
echo "   ✅ run_formation_docker.sh - Quick start script"
echo "   ✅ test_formation_simple.py - Simple formation test (no ROS 2)"
echo ""
echo "🚀 Quick Start Options:"
echo ""
echo "Option 1: Simple Formation Test (No ROS 2 required)"
echo "   python3 test_formation_simple.py"
echo ""
echo "Option 2: Full ROS 2 + Gazebo Simulation (Docker)"
echo "   ./run_formation_docker.sh"
echo ""
echo "Option 3: Manual Docker Setup"
echo "   1. ./setup_docker_workspace.sh"
echo "   2. docker-compose -f docker-compose-ros2-gazebo.yml up -d"
echo "   3. Follow the instructions in the script output"
echo ""
echo "🔧 Troubleshooting:"
echo "- Make sure Docker Desktop is running"
echo "- If containers fail to start, check: docker logs ros2_gazebo_formation"
echo "- For X11 issues on macOS, install XQuartz: brew install --cask xquartz"
echo ""
echo "🚁 Happy Formation Flying! 🚁"
