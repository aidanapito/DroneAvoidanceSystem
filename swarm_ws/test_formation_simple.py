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
