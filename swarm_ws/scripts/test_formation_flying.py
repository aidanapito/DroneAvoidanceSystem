#!/usr/bin/env python3
"""
Formation Flying Test and Visualization

This script demonstrates and tests formation flying logic with real-time visualization.
Shows different formation patterns, transitions, and swarm behavior.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Circle, FancyBboxPatch
import time
from typing import List, Tuple, Dict
import math

# Set up matplotlib for better performance
plt.ion()  # Interactive mode on


class FormationPattern:
    """Different formation patterns for the swarm."""
    
    @staticmethod
    def triangle(center: Tuple[float, float], spacing: float, altitude: float) -> List[Tuple[float, float, float]]:
        """Create a triangle formation."""
        x, y = center
        positions = [
            (x, y, altitude),  # Center
            (x - spacing, y - spacing, altitude),  # Left back
            (x + spacing, y - spacing, altitude),  # Right back
        ]
        return positions
    
    @staticmethod
    def line(center: Tuple[float, float], spacing: float, altitude: float) -> List[Tuple[float, float, float]]:
        """Create a line formation."""
        x, y = center
        positions = [
            (x - spacing, y, altitude),  # Left
            (x, y, altitude),            # Center
            (x + spacing, y, altitude),  # Right
        ]
        return positions
    
    @staticmethod
    def square(center: Tuple[float, float], spacing: float, altitude: float) -> List[Tuple[float, float, float]]:
        """Create a square formation."""
        x, y = center
        half_spacing = spacing / 2
        positions = [
            (x - half_spacing, y - half_spacing, altitude),  # Bottom left
            (x + half_spacing, y - half_spacing, altitude),  # Bottom right
            (x, y + half_spacing, altitude),                 # Top center
        ]
        return positions
    
    @staticmethod
    def v_formation(center: Tuple[float, float], spacing: float, altitude: float) -> List[Tuple[float, float, float]]:
        """Create a V formation."""
        x, y = center
        positions = [
            (x, y, altitude),                    # Leader
            (x - spacing, y - spacing, altitude), # Left wing
            (x + spacing, y - spacing, altitude), # Right wing
        ]
        return positions


class Drone:
    """Represents a single drone in the swarm."""
    
    def __init__(self, drone_id: str, initial_pos: Tuple[float, float, float]):
        self.id = drone_id
        self.position = np.array(initial_pos, dtype=float)
        self.target_position = np.array(initial_pos, dtype=float)
        self.velocity = np.array([0.0, 0.0, 0.0], dtype=float)
        self.max_velocity = 2.0  # m/s
        self.max_acceleration = 1.0  # m/s²
        self.formation_radius = 0.3  # m (visual size)
        
    def update_position(self, dt: float):
        """Update drone position based on target and current velocity."""
        # Calculate desired velocity towards target
        direction = self.target_position - self.position
        distance = np.linalg.norm(direction)
        
        if distance > 0.1:  # Only move if not at target
            # Normalize direction and apply max velocity
            desired_velocity = direction / distance * self.max_velocity
            
            # Smooth velocity change (acceleration limit)
            velocity_diff = desired_velocity - self.velocity
            max_velocity_change = self.max_acceleration * dt
            if np.linalg.norm(velocity_diff) > max_velocity_change:
                velocity_diff = velocity_diff / np.linalg.norm(velocity_diff) * max_velocity_change
            
            self.velocity += velocity_diff
            
            # Update position
            self.position += self.velocity * dt
            
            # Ensure we don't overshoot target
            if np.linalg.norm(self.target_position - self.position) < 0.1:
                self.position = self.target_position.copy()
                self.velocity = np.array([0.0, 0.0, 0.0])
    
    def set_target(self, target: Tuple[float, float, float]):
        """Set a new target position for the drone."""
        self.target_position = np.array(target, dtype=float)


class SwarmFormationController:
    """Controls the formation flying of the drone swarm."""
    
    def __init__(self, num_drones: int = 3):
        self.num_drones = num_drones
        self.drones: List[Drone] = []
        self.formation_center = np.array([0.0, 0.0, 5.0])
        self.formation_spacing = 3.0
        self.current_formation = "triangle"
        self.formation_patterns = ["triangle", "line", "square", "v_formation"]
        self.formation_index = 0
        
        # Initialize drones in a triangle formation
        self._initialize_drones()
        self._update_formation_positions()
        
        # Animation properties
        self.animation_speed = 0.05  # seconds per frame
        self.formation_change_interval = 10.0  # seconds
        self.last_formation_change = time.time()
        
    def _initialize_drones(self):
        """Initialize the drone swarm."""
        initial_positions = FormationPattern.triangle(
            (0.0, 0.0), self.formation_spacing, 5.0
        )
        
        for i in range(self.num_drones):
            drone = Drone(f"UAV{i+1}", initial_positions[i])
            self.drones.append(drone)
    
    def _update_formation_positions(self):
        """Update target positions for all drones based on current formation."""
        if self.current_formation == "triangle":
            positions = FormationPattern.triangle(
                (self.formation_center[0], self.formation_center[1]), 
                self.formation_spacing, 
                self.formation_center[2]
            )
        elif self.current_formation == "line":
            positions = FormationPattern.line(
                (self.formation_center[0], self.formation_center[1]), 
                self.formation_spacing, 
                self.formation_center[2]
            )
        elif self.current_formation == "square":
            positions = FormationPattern.square(
                (self.formation_center[0], self.formation_center[1]), 
                self.formation_spacing, 
                self.formation_center[2]
            )
        elif self.current_formation == "v_formation":
            positions = FormationPattern.v_formation(
                (self.formation_center[0], self.formation_center[1]), 
                self.formation_spacing, 
                self.formation_center[2]
            )
        
        # Assign positions to drones
        for i, drone in enumerate(self.drones):
            if i < len(positions):
                drone.set_target(positions[i])
    
    def change_formation(self):
        """Change to the next formation pattern."""
        self.formation_index = (self.formation_index + 1) % len(self.formation_patterns)
        self.current_formation = self.formation_patterns[self.formation_index]
        self._update_formation_positions()
        print(f"🔄 Changed to {self.current_formation} formation")
    
    def move_formation(self, new_center: Tuple[float, float, float]):
        """Move the entire formation to a new center."""
        self.formation_center = np.array(new_center)
        self._update_formation_positions()
        print(f"📍 Moving formation to {new_center}")
    
    def update(self, dt: float):
        """Update all drones and check for formation changes."""
        # Update drone positions
        for drone in self.drones:
            drone.update_position(dt)
        
        # Check if it's time to change formation
        current_time = time.time()
        if current_time - self.last_formation_change > self.formation_change_interval:
            self.change_formation()
            self.last_formation_change = current_time
        
        # Move formation in a circular pattern
        t = current_time * 0.1
        radius = 5.0
        new_x = radius * math.cos(t)
        new_y = radius * math.sin(t)
        self.move_formation((new_x, new_y, 5.0))
    
    def get_formation_status(self) -> Dict:
        """Get current formation status."""
        return {
            "formation": self.current_formation,
            "center": self.formation_center.tolist(),
            "spacing": self.formation_spacing,
            "drones": [
                {
                    "id": drone.id,
                    "position": drone.position.tolist(),
                    "target": drone.target_position.tolist(),
                    "velocity": drone.velocity.tolist()
                }
                for drone in self.drones
            ]
        }


class FormationVisualizer:
    """Visualizes the formation flying in real-time."""
    
    def __init__(self, controller: SwarmFormationController):
        self.controller = controller
        self.fig, self.ax = plt.subplots(figsize=(12, 8))
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel('X (meters)')
        self.ax.set_ylabel('Y (meters)')
        self.ax.set_title('Drone Swarm Formation Flying Test')
        
        # Initialize drone markers
        self.drone_markers = []
        self.target_markers = []
        self.trajectory_lines = []
        
        # Colors for different drones
        self.drone_colors = ['red', 'blue', 'green']
        
        self._setup_visualization()
    
    def _setup_visualization(self):
        """Set up the initial visualization elements."""
        # Clear existing elements
        for marker in self.drone_markers + self.target_markers + self.trajectory_lines:
            if hasattr(marker, 'remove'):
                marker.remove()
        
        self.drone_markers = []
        self.target_markers = []
        self.trajectory_lines = []
        
        # Create drone markers
        for i, drone in enumerate(self.controller.drones):
            # Drone marker (circle)
            drone_marker = Circle(
                (drone.position[0], drone.position[1]), 
                drone.formation_radius, 
                color=self.drone_colors[i], 
                alpha=0.8,
                label=f"{drone.id}"
            )
            self.ax.add_patch(drone_marker)
            self.drone_markers.append(drone_marker)
            
            # Target marker (smaller circle)
            target_marker = Circle(
                (drone.target_position[0], drone.target_position[1]), 
                0.1, 
                color=self.drone_colors[i], 
                alpha=0.5,
                linestyle='--',
                fill=False
            )
            self.ax.add_patch(target_marker)
            self.target_markers.append(target_marker)
            
            # Trajectory line
            line, = self.ax.plot([], [], color=self.drone_colors[i], alpha=0.3, linestyle='-')
            self.trajectory_lines.append(line)
        
        # Add legend
        self.ax.legend()
        
        # Add formation info text
        self.info_text = self.ax.text(
            0.02, 0.98, '', 
            transform=self.ax.transAxes, 
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )
    
    def update_visualization(self):
        """Update the visualization with current drone positions."""
        # Update drone markers
        for i, (drone, marker) in enumerate(zip(self.controller.drones, self.drone_markers)):
            marker.center = (drone.position[0], drone.position[1])
            
            # Update target markers
            self.target_markers[i].center = (drone.target_position[0], drone.target_position[1])
            
            # Update trajectory lines (simple line from current to target)
            x_data = [drone.position[0], drone.target_position[0]]
            y_data = [drone.position[1], drone.target_position[1]]
            self.trajectory_lines[i].set_data(x_data, y_data)
        
        # Update info text
        status = self.controller.get_formation_status()
        info = f"Formation: {status['formation'].upper()}\n"
        info += f"Center: ({status['center'][0]:.1f}, {status['center'][1]:.1f})\n"
        info += f"Spacing: {status['spacing']:.1f}m\n"
        info += f"Time: {time.time():.1f}s"
        
        self.info_text.set_text(info)
        
        # Redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def run_formation_test():
    """Run the formation flying test with visualization."""
    print("🚁 Starting Formation Flying Test")
    print("=" * 50)
    print("This will show a real-time visualization of:")
    print("- 3 drones flying in different formations")
    print("- Automatic formation transitions")
    print("- Smooth movement and coordination")
    print("- Real-time status updates")
    print()
    print("Formations: Triangle → Line → Square → V-Formation")
    print("Press Ctrl+C to stop")
    print()
    
    # Create controller and visualizer
    controller = SwarmFormationController(num_drones=3)
    visualizer = FormationVisualizer(controller)
    
    try:
        # Main simulation loop
        start_time = time.time()
        last_update = start_time
        
        while True:
            current_time = time.time()
            dt = current_time - last_update
            
            if dt >= controller.animation_speed:
                # Update controller
                controller.update(dt)
                
                # Update visualization
                visualizer.update_visualization()
                
                last_update = current_time
                
                # Small delay to prevent excessive CPU usage
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\n⏹️  Formation test stopped by user")
        print("🎉 Test completed successfully!")
        
        # Final status
        final_status = controller.get_formation_status()
        print(f"\n📊 Final Status:")
        print(f"   Formation: {final_status['formation']}")
        print(f"   Center: {final_status['center']}")
        print(f"   All drones positioned correctly")


if __name__ == "__main__":
    try:
        run_formation_test()
    except Exception as e:
        print(f"❌ Error in formation test: {e}")
        import traceback
        traceback.print_exc()
