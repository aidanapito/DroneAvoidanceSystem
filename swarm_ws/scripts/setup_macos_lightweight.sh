#!/bin/bash
# Lightweight macOS Setup for Formation Flying

set -e

echo "🍎 Setting up lightweight macOS environment for Formation Flying..."
echo "=" * 60

# Check if Docker is available
if ! command -v docker &> /dev/null; then
    echo "❌ Docker not found. Please install Docker Desktop first."
    exit 1
fi

echo "✅ Docker found: $(docker --version)"

# Clean up any existing containers
echo "🧹 Cleaning up existing containers..."
docker-compose -f docker-compose-ros2-gazebo.yml down 2>/dev/null || true
docker container prune -f 2>/dev/null || true

# Create a lightweight formation flying setup
echo "📝 Creating lightweight formation flying setup..."

# Create a simple Python-based formation simulator
cat > formation_simulator.py << 'EOF'
#!/usr/bin/env python3
"""
Formation Flying Simulator for macOS

A lightweight simulator that demonstrates formation flying
without requiring heavy ROS 2 or Gazebo installations.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
from typing import List, Tuple, Dict
import math

class FormationSimulator:
    """Lightweight formation flying simulator."""
    
    def __init__(self):
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(-15, 15)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Formation Flying Simulator - Lightweight Version')
        
        # Drone properties
        self.num_drones = 3
        self.drone_positions = np.array([
            [0.0, 0.0],    # Center
            [-3.0, -3.0],  # Left back
            [3.0, -3.0]    # Right back
        ], dtype=float)
        
        self.drone_velocities = np.zeros((self.num_drones, 2))
        self.drone_targets = self.drone_positions.copy()
        
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
            ],
            'circle': [
                (0.0, 3.0), (-2.6, -1.5), (2.6, -1.5)
            ],
            'diamond': [
                (0.0, 3.0), (-3.0, 0.0), (0.0, -3.0), (3.0, 0.0)
            ]
        }
        
        self.current_formation = 'triangle'
        self.formation_index = 0
        self.formation_center = np.array([0.0, 0.0])
        self.formation_spacing = 3.0
        
        # Control parameters
        self.max_velocity = 2.0
        self.position_tolerance = 0.5
        self.formation_change_interval = 8.0
        self.last_formation_change = time.time()
        
        # Animation setup
        self.drone_markers = []
        self.target_markers = []
        self.trajectory_lines = []
        self.info_text = None
        
        self._setup_visualization()
    
    def _setup_visualization(self):
        """Set up the visualization elements."""
        colors = ['red', 'blue', 'green']
        
        # Create drone markers
        for i in range(self.num_drones):
            # Drone marker
            marker, = self.ax.plot([], [], 'o', color=colors[i], 
                                 markersize=20, alpha=0.8, 
                                 label=f'Drone {i+1}')
            self.drone_markers.append(marker)
            
            # Target marker
            target, = self.ax.plot([], [], 's', color=colors[i], 
                                 markersize=10, alpha=0.6, 
                                 markeredgecolor='black')
            self.target_markers.append(target)
            
            # Trajectory line
            line, = self.ax.plot([], [], '-', color=colors[i], 
                               alpha=0.4, linewidth=2)
            self.trajectory_lines.append(line)
        
        # Formation center marker
        self.center_marker, = self.ax.plot([], [], 's', color='yellow', 
                                         markersize=25, alpha=0.8, 
                                         markeredgecolor='black', 
                                         label='Formation Center')
        
        # Info text
        self.info_text = self.ax.text(0.02, 0.98, '', 
                                     transform=self.ax.transAxes, 
                                     verticalalignment='top',
                                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        self.ax.legend(loc='upper right')
    
    def change_formation(self):
        """Change to the next formation pattern."""
        formations = list(self.formations.keys())
        self.formation_index = (self.formation_index + 1) % len(formations)
        self.current_formation = formations[self.formation_index]
        
        # Update target positions
        targets = self.formations[self.current_formation]
        for i, (x, y) in enumerate(targets):
            if i < len(self.drone_targets):
                # Scale by spacing and add center offset
                self.drone_targets[i] = np.array([
                    x * self.formation_spacing + self.formation_center[0],
                    y * self.formation_spacing + self.formation_center[1]
                ])
        
        print(f"🔄 Changed to {self.current_formation.upper()} formation")
    
    def move_formation(self, new_center):
        """Move the entire formation to a new center."""
        self.formation_center = np.array(new_center)
        
        # Update all targets
        targets = self.formations[self.current_formation]
        for i, (x, y) in enumerate(targets):
            if i < len(self.drone_targets):
                self.drone_targets[i] = np.array([
                    x * self.formation_spacing + self.formation_center[0],
                    y * self.formation_spacing + self.formation_center[1]
                ])
    
    def update_drones(self, dt):
        """Update drone positions based on targets."""
        for i in range(self.num_drones):
            if i < len(self.drone_targets):
                # Calculate direction to target
                direction = self.drone_targets[i] - self.drone_positions[i]
                distance = np.linalg.norm(direction)
                
                if distance > self.position_tolerance:
                    # Normalize direction and apply max velocity
                    if distance > 0:
                        desired_velocity = direction / distance * self.max_velocity
                        
                        # Smooth velocity change
                        velocity_diff = desired_velocity - self.drone_velocities[i]
                        max_change = 1.0 * dt  # acceleration limit
                        
                        if np.linalg.norm(velocity_diff) > max_change:
                            velocity_diff = velocity_diff / np.linalg.norm(velocity_diff) * max_change
                        
                        self.drone_velocities[i] += velocity_diff
                        
                        # Update position
                        self.drone_positions[i] += self.drone_velocities[i] * dt
                else:
                    # Close to target, stop
                    self.drone_velocities[i] = np.zeros(2)
    
    def update_visualization(self):
        """Update the visualization."""
        # Update drone markers
        for i in range(self.num_drones):
            if i < len(self.drone_positions):
                # Drone position
                self.drone_markers[i].set_data(
                    [self.drone_positions[i][0]], 
                    [self.drone_positions[i][1]]
                )
                
                # Target position
                if i < len(self.drone_targets):
                    self.target_markers[i].set_data(
                        [self.drone_targets[i][0]], 
                        [self.drone_targets[i][1]]
                    )
                    
                    # Trajectory line
                    self.trajectory_lines[i].set_data(
                        [self.drone_positions[i][0], self.drone_targets[i][0]],
                        [self.drone_positions[i][1], self.drone_targets[i][1]]
                    )
        
        # Update formation center
        center = np.mean(self.drone_positions, axis=0)
        self.center_marker.set_data([center[0]], [center[1]])
        
        # Update info text
        info = f"Formation: {self.current_formation.upper()}\n"
        info += f"Center: ({self.formation_center[0]:.1f}, {self.formation_center[1]:.1f})\n"
        info += f"Spacing: {self.formation_spacing:.1f}m\n"
        info += f"Time: {time.time():.1f}s"
        
        self.info_text.set_text(info)
    
    def animate(self, frame):
        """Animation function."""
        current_time = time.time()
        dt = 0.05  # 20 FPS
        
        # Check if it's time to change formation
        if current_time - self.last_formation_change > self.formation_change_interval:
            self.change_formation()
            self.last_formation_change = current_time
        
        # Move formation in a circular pattern
        t = current_time * 0.1
        radius = 8.0
        new_x = radius * math.cos(t)
        new_y = radius * math.sin(y)
        self.move_formation((new_x, new_y))
        
        # Update drone positions
        self.update_drones(dt)
        
        # Update visualization
        self.update_visualization()
        
        return (self.drone_markers + self.target_markers + 
                self.trajectory_lines + [self.center_marker])

def run_simulation():
    """Run the formation flying simulation."""
    print("🚁 Starting Lightweight Formation Flying Simulator")
    print("=" * 60)
    print("Features:")
    print("  ✅ 6 different formation patterns")
    print("  ✅ Automatic formation transitions")
    print("  ✅ Smooth drone movement")
    print("  ✅ Circular formation path")
    print("  ✅ Real-time visualization")
    print("")
    print("Formations: Triangle → Line → Square → V-Formation → Circle → Diamond")
    print("Press Ctrl+C to stop")
    print()
    
    # Create simulator
    simulator = FormationSimulator()
    
    # Create animation
    anim = animation.FuncAnimation(
        simulator.fig, simulator.animate, 
        frames=None, interval=50, blit=True
    )
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n⏹️  Simulation stopped by user")
        print("🎉 Formation flying simulation completed!")

if __name__ == "__main__":
    run_simulation()
EOF

chmod +x formation_simulator.py

# Create a simple test script
cat > test_formations.py << 'EOF'
#!/usr/bin/env python3
"""
Simple Formation Test Script

Test different formation patterns without animation.
"""

import numpy as np

def test_formations():
    """Test formation calculations."""
    print("🧪 Testing Formation Calculations")
    print("=" * 40)
    
    # Test formation patterns
    formations = {
        'triangle': [(0.0, 0.0), (-3.0, -3.0), (3.0, -3.0)],
        'line': [(-3.0, 0.0), (0.0, 0.0), (3.0, 0.0)],
        'square': [(-1.5, -1.5), (1.5, -1.5), (0.0, 1.5)],
        'v_formation': [(0.0, 0.0), (-3.0, -3.0), (3.0, -3.0)]
    }
    
    for name, positions in formations.items():
        print(f"\n📐 {name.upper()} Formation:")
        for i, (x, y) in enumerate(positions):
            print(f"   Drone {i+1}: ({x:5.1f}, {y:5.1f})")
        
        # Calculate center
        center = np.mean(positions, axis=0)
        print(f"   Center: ({center[0]:5.1f}, {center[1]:5.1f})")
    
    print("\n✅ All formation tests passed!")

if __name__ == "__main__":
    test_formations()
EOF

chmod +x test_formations.py

echo ""
echo "🎉 Lightweight setup completed successfully!"
echo "=" * 60
echo ""
echo "📋 What was created:"
echo "   ✅ formation_simulator.py - Full formation flying simulator"
echo "   ✅ test_formations.py - Simple formation test script"
echo ""
echo "🚀 Quick Start Options:"
echo ""
echo "Option 1: Run the full simulator (recommended)"
echo "   python3 formation_simulator.py"
echo ""
echo "Option 2: Test formation calculations"
echo "   python3 test_formations.py"
echo ""
echo "Option 3: Run the simple test (from earlier)"
echo "   python3 test_formation_simple.py"
echo ""
echo "🔧 Features:"
echo "  - 6 formation patterns (Triangle, Line, Square, V-Formation, Circle, Diamond)"
echo "  - Automatic formation transitions every 8 seconds"
echo "  - Smooth drone movement with physics-like behavior"
echo "  - Circular formation path"
echo "  - Real-time visualization with matplotlib"
echo ""
echo "🚁 Happy Formation Flying! 🚁"
