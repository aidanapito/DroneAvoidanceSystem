#!/usr/bin/env python3
"""
Professional Formation Flying Simulator for macOS

A high-quality simulator that demonstrates formation flying
with 3D visualization, physics-based movement, and interactive controls.
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import time
from typing import List, Tuple, Dict
import math

class ProfessionalFormationSimulator:
    """Professional-grade formation flying simulator."""
    
    def __init__(self):
        # Create 3D figure
        self.fig = plt.figure(figsize=(14, 10))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Set up 3D view
        self.ax.set_xlim(-15, 15)
        self.ax.set_ylim(-15, 15)
        self.ax.set_zlim(0, 10)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_zlabel('Z (meters)')
        self.ax.set_title('Professional Formation Flying Simulator', fontsize=16, fontweight='bold')
        
        # Drone properties
        self.num_drones = 3
        self.drone_positions = np.array([
            [0.0, 0.0, 5.0],     # Center
            [-3.0, -3.0, 5.0],   # Left back
            [3.0, -3.0, 5.0]     # Right back
        ], dtype=float)
        
        self.drone_velocities = np.zeros((self.num_drones, 3))
        self.drone_targets = self.drone_positions.copy()
        
        # Formation patterns with 3D coordinates
        self.formations = {
            'triangle': [
                (0.0, 0.0, 0.0), (-3.0, -3.0, 0.0), (3.0, -3.0, 0.0)
            ],
            'line': [
                (-3.0, 0.0, 0.0), (0.0, 0.0, 0.0), (3.0, 0.0, 0.0)
            ],
            'square': [
                (-1.5, -1.5, 0.0), (1.5, -1.5, 0.0), (0.0, 1.5, 0.0)
            ],
            'v_formation': [
                (0.0, 0.0, 0.0), (-3.0, -3.0, 0.0), (3.0, -3.0, 0.0)
            ],
            'circle': [
                (0.0, 3.0, 0.0), (-2.6, -1.5, 0.0), (2.6, -1.5, 0.0)
            ],
            'diamond': [
                (0.0, 3.0, 0.0), (-3.0, 0.0, 0.0), (0.0, -3.0, 0.0), (3.0, 0.0, 0.0)
            ]
        }
        
        self.current_formation = 'triangle'
        self.formation_index = 0
        self.formation_center = np.array([0.0, 0.0, 5.0])
        self.formation_spacing = 3.0
        self.formation_altitude = 5.0
        
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
        self.formation_text = None
        
        # Performance tracking
        self.frame_count = 0
        self.start_time = time.time()
        
        self._setup_visualization()
        
    def _setup_visualization(self):
        """Set up the 3D visualization elements."""
        colors = ['red', 'blue', 'green']
        
        # Create drone markers (3D spheres)
        for i in range(self.num_drones):
            # Drone marker
            marker, = self.ax.plot([], [], [], 'o', color=colors[i], 
                                 markersize=15, alpha=0.8, 
                                 label=f'Drone {i+1}')
            self.drone_markers.append(marker)
            
            # Target marker
            target, = self.ax.plot([], [], [], 's', color=colors[i], 
                                 markersize=8, alpha=0.6, 
                                 markeredgecolor='black')
            self.target_markers.append(target)
            
            # Trajectory line
            line, = self.ax.plot([], [], [], '-', color=colors[i], 
                               alpha=0.4, linewidth=2)
            self.trajectory_lines.append(line)
        
        # Formation center marker
        self.center_marker, = self.ax.plot([], [], [], 's', color='yellow', 
                                         markersize=20, alpha=0.8, 
                                         markeredgecolor='black', 
                                         label='Formation Center')
        
        # Info text
        self.info_text = self.ax.text2D(0.02, 0.98, '', 
                                       transform=self.ax.transAxes, 
                                       verticalalignment='top',
                                       bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        # Formation info text
        self.formation_text = self.ax.text2D(0.02, 0.92, '', 
                                           transform=self.ax.transAxes, 
                                           verticalalignment='top',
                                           bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
        
        # Add legend
        self.ax.legend(loc='upper right')
        
        # Set initial view angle
        self.ax.view_init(elev=20, azim=45)
    
    def change_formation(self):
        """Change to the next formation pattern."""
        formations = list(self.formations.keys())
        self.formation_index = (self.formation_index + 1) % len(formations)
        self.current_formation = formations[self.formation_index]
        
        # Update target positions
        targets = self.formations[self.current_formation]
        for i, (x, y, z) in enumerate(targets):
            if i < len(self.drone_targets):
                # Scale by spacing and add center offset
                self.drone_targets[i] = np.array([
                    x * self.formation_spacing + self.formation_center[0],
                    y * self.formation_spacing + self.formation_center[1],
                    z + self.formation_altitude
                ])
        
        print(f"🔄 Changed to {self.current_formation.upper()} formation")
    
    def move_formation(self, new_center):
        """Move the entire formation to a new center."""
        self.formation_center = np.array(new_center)
        
        # Update all targets
        targets = self.formations[self.current_formation]
        for i, (x, y, z) in enumerate(targets):
            if i < len(self.drone_targets):
                self.drone_targets[i] = np.array([
                    x * self.formation_spacing + self.formation_center[0],
                    y * self.formation_spacing + self.formation_center[1],
                    z + self.formation_altitude
                ])
    
    def update_drones(self, dt):
        """Update drone positions based on targets with physics."""
        for i in range(self.num_drones):
            if i < len(self.drone_targets):
                # Calculate direction to target
                direction = self.drone_targets[i] - self.drone_positions[i]
                distance = np.linalg.norm(direction)
                
                if distance > self.position_tolerance:
                    # Normalize direction and apply max velocity
                    if distance > 0:
                        desired_velocity = direction / distance * self.max_velocity
                        
                        # Smooth velocity change (acceleration limit)
                        velocity_diff = desired_velocity - self.drone_velocities[i]
                        max_change = 1.5 * dt  # acceleration limit
                        
                        if np.linalg.norm(velocity_diff) > max_change:
                            velocity_diff = velocity_diff / np.linalg.norm(velocity_diff) * max_change
                        
                        self.drone_velocities[i] += velocity_diff
                        
                        # Update position
                        self.drone_positions[i] += self.drone_velocities[i] * dt
                else:
                    # Close to target, gradually stop
                    self.drone_velocities[i] *= 0.9
    
    def update_visualization(self):
        """Update the 3D visualization."""
        # Update drone markers
        for i in range(self.num_drones):
            if i < len(self.drone_positions):
                # Drone position
                self.drone_markers[i].set_data(
                    [self.drone_positions[i][0]], 
                    [self.drone_positions[i][1]]
                )
                self.drone_markers[i].set_3d_properties([self.drone_positions[i][2]])
                
                # Target position
                if i < len(self.drone_targets):
                    self.target_markers[i].set_data(
                        [self.drone_targets[i][0]], 
                        [self.drone_targets[i][1]]
                    )
                    self.target_markers[i].set_3d_properties([self.drone_targets[i][2]])
                    
                    # Trajectory line
                    self.trajectory_lines[i].set_data(
                        [self.drone_positions[i][0], self.drone_targets[i][0]],
                        [self.drone_positions[i][1], self.drone_targets[i][1]]
                    )
                    self.trajectory_lines[i].set_3d_properties(
                        [self.drone_positions[i][2], self.drone_targets[i][2]]
                    )
        
        # Update formation center
        self.center_marker.set_data([self.formation_center[0]], [self.formation_center[1]])
        self.center_marker.set_3d_properties([self.formation_center[2]])
        
        # Update info text
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
        
        info = f"Time: {elapsed_time:.1f}s | FPS: {fps:.1f} | Drones: {self.num_drones}"
        self.info_text.set_text(info)
        
        # Update formation info
        formation_info = f"Formation: {self.current_formation.upper()}\n"
        formation_info += f"Center: ({self.formation_center[0]:.1f}, {self.formation_center[1]:.1f}, {self.formation_center[2]:.1f})\n"
        formation_info += f"Spacing: {self.formation_spacing:.1f}m"
        
        self.formation_text.set_text(formation_info)
    
    def animate(self, frame):
        """Animation function for real-time updates."""
        current_time = time.time()
        dt = 0.05  # 20 FPS
        
        # Check if it's time to change formation
        if current_time - self.last_formation_change > self.formation_change_interval:
            self.change_formation()
            self.last_formation_change = current_time
        
        # Move formation in a complex 3D pattern
        t = current_time * 0.1
        radius = 8.0
        height_variation = 2.0
        
        # Circular motion with height variation
        new_x = radius * math.cos(t)
        new_y = radius * math.sin(t)
        new_z = 5.0 + height_variation * math.sin(t * 0.5)
        
        self.move_formation((new_x, new_y, new_z))
        
        # Update drone positions
        self.update_drones(dt)
        
        # Update visualization
        self.update_visualization()
        
        # Update frame counter
        self.frame_count += 1
        
        # Rotate view slowly for dynamic effect
        if frame % 100 == 0:  # Every 5 seconds
            current_azim = self.ax.azim
            self.ax.view_init(elev=20, azim=current_azim + 5)
        
        return (self.drone_markers + self.target_markers + 
                self.trajectory_lines + [self.center_marker])

def run_professional_simulation():
    """Run the professional formation flying simulation."""
    print("🚁 Starting Professional Formation Flying Simulator")
    print("=" * 60)
    print("Features:")
    print("  ✅ 6 different 3D formation patterns")
    print("  ✅ Automatic formation transitions every 8 seconds")
    print("  ✅ Physics-based drone movement with acceleration limits")
    print("  ✅ 3D visualization with rotating camera view")
    print("  ✅ Complex formation path (circular + height variation)")
    print("  ✅ Real-time performance metrics (FPS, timing)")
    print("  ✅ Professional-grade interface")
    print("")
    print("Formations: Triangle → Line → Square → V-Formation → Circle → Diamond")
    print("Controls: Mouse to rotate view, scroll to zoom")
    print("Press Ctrl+C to stop")
    print()
    
    # Create simulator
    simulator = ProfessionalFormationSimulator()
    
    # Create animation
    anim = animation.FuncAnimation(
        simulator.fig, simulator.animate, 
        frames=None, interval=50, blit=True
    )
    
    try:
        plt.show()
    except KeyboardInterrupt:
        print("\n⏹️  Simulation stopped by user")
        
        # Final statistics
        elapsed_time = time.time() - simulator.start_time
        fps = simulator.frame_count / elapsed_time if elapsed_time > 0 else 0
        
        print(f"\n📊 Final Statistics:")
        print(f"   Total time: {elapsed_time:.1f} seconds")
        print(f"   Total frames: {simulator.frame_count}")
        print(f"   Average FPS: {fps:.1f}")
        print(f"   Final formation: {simulator.current_formation.upper()}")
        
        print("🎉 Professional formation flying simulation completed!")

if __name__ == "__main__":
    run_professional_simulation()
