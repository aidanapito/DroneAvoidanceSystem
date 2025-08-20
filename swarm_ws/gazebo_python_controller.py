#!/usr/bin/env python3
"""
Gazebo Python Formation Controller
Controls drones in Gazebo for formation flying without ROS 2
"""

import time
import math
import subprocess
import json
from typing import List, Tuple

class GazeboFormationController:
    def __init__(self):
        self.drones = {
            'drone1': {'pose': [0, 0, 1], 'color': 'red'},
            'drone2': {'pose': [-1, 0, 1], 'color': 'blue'}, 
            'drone3': {'pose': [1, 0, 1], 'color': 'green'}
        }
        
        self.formation_patterns = {
            'triangle': [
                [0, 0, 1],      # Center
                [-0.5, -0.866, 1],  # Left
                [0.5, -0.866, 1]    # Right
            ],
            'line': [
                [0, 0, 1],      # Center
                [-1, 0, 1],     # Left
                [1, 0, 1]       # Right
            ],
            'square': [
                [0, 0, 1],      # Center
                [-0.5, -0.5, 1],    # Bottom Left
                [0.5, -0.5, 1],     # Bottom Right
                [0, 0.5, 1]         # Top
            ],
            'v_formation': [
                [0, 0, 1],      # Leader
                [-0.5, -0.5, 1],    # Left Wing
                [0.5, -0.5, 1]      # Right Wing
            ]
        }
        
        self.current_formation = 'triangle'
        self.formation_center = [0, 0, 1]
        
    def send_gazebo_command(self, model_name: str, pose: List[float]):
        """Send a pose command to a drone in Gazebo"""
        try:
            # Use gz service to set model pose
            cmd = [
                'gz', 'service', '--service', '/gazebo/set_entity_state',
                '--request', f'name: "{model_name}", pose: {{position: {{x: {pose[0]}, y: {pose[1]}, z: {pose[2]}}}}}'
            ]
            subprocess.run(cmd, check=True, capture_output=True)
            print(f"✅ Moved {model_name} to {pose}")
        except subprocess.CalledProcessError as e:
            print(f"⚠️  Could not move {model_name}: {e}")
            # Fallback: try using gz model command
            try:
                cmd = ['gz', 'model', '--move', model_name, '--pose', f'{pose[0]},{pose[1]},{pose[2]}']
                subprocess.run(cmd, check=True, capture_output=True)
                print(f"✅ Moved {model_name} using fallback method")
            except subprocess.CalledProcessError:
                print(f"❌ Failed to move {model_name}")
    
    def change_formation(self, formation_name: str):
        """Change to a different formation pattern"""
        if formation_name not in self.formation_patterns:
            print(f"❌ Unknown formation: {formation_name}")
            return
            
        self.current_formation = formation_name
        print(f"🔄 Changing to {formation_name} formation...")
        
        # Get formation positions relative to center
        positions = self.formation_patterns[formation_name]
        
        # Move each drone to its new position
        drone_names = list(self.drones.keys())
        for i, (drone_name, position) in enumerate(zip(drone_names, positions)):
            if i < len(drone_names):
                new_pose = [
                    self.formation_center[0] + position[0],
                    self.formation_center[1] + position[1], 
                    self.formation_center[2] + position[2]
                ]
                self.send_gazebo_command(drone_name, new_pose)
                time.sleep(0.5)  # Small delay between moves
        
        print(f"✅ Formation changed to {formation_name}")
    
    def move_formation(self, dx: float, dy: float, dz: float):
        """Move the entire formation by offset"""
        print(f"🚁 Moving formation by [{dx}, {dy}, {dz}]")
        
        # Update formation center
        self.formation_center[0] += dx
        self.formation_center[1] += dy
        self.formation_center[2] += dz
        
        # Get current formation pattern
        positions = self.formation_patterns[self.current_formation]
        
        # Move each drone
        drone_names = list(self.drones.keys())
        for i, (drone_name, position) in enumerate(zip(drone_names, positions)):
            if i < len(drone_names):
                new_pose = [
                    self.formation_center[0] + position[0],
                    self.formation_center[1] + position[1],
                    self.formation_center[2] + position[2]
                ]
                self.send_gazebo_command(drone_name, new_pose)
                time.sleep(0.2)
        
        print(f"✅ Formation moved to {self.formation_center}")
    
    def demo_formation_changes(self):
        """Demonstrate different formations"""
        print("🎬 Starting formation demonstration...")
        
        formations = ['triangle', 'line', 'square', 'v_formation']
        
        for formation in formations:
            print(f"\n🔄 Changing to {formation} formation...")
            self.change_formation(formation)
            time.sleep(3)  # Hold formation for 3 seconds
            
            # Move formation around
            print("🚁 Moving formation...")
            self.move_formation(0.5, 0.5, 0)
            time.sleep(2)
            self.move_formation(-0.5, -0.5, 0)
            time.sleep(2)
        
        print("🎉 Formation demonstration complete!")
    
    def interactive_control(self):
        """Interactive formation control"""
        print("\n🎮 Interactive Formation Control")
        print("Commands:")
        print("  formation <name> - Change formation (triangle, line, square, v_formation)")
        print("  move <dx> <dy> <dz> - Move formation by offset")
        print("  demo - Run formation demonstration")
        print("  quit - Exit")
        
        while True:
            try:
                command = input("\n> ").strip().lower().split()
                if not command:
                    continue
                    
                if command[0] == 'formation' and len(command) > 1:
                    self.change_formation(command[1])
                elif command[0] == 'move' and len(command) > 3:
                    dx, dy, dz = float(command[1]), float(command[2]), float(command[3])
                    self.move_formation(dx, dy, dz)
                elif command[0] == 'demo':
                    self.demo_formation_changes()
                elif command[0] == 'quit':
                    print("👋 Goodbye!")
                    break
                else:
                    print("❌ Unknown command. Type 'help' for commands.")
                    
            except KeyboardInterrupt:
                print("\n👋 Goodbye!")
                break
            except Exception as e:
                print(f"❌ Error: {e}")

def main():
    print("🚁 Gazebo Python Formation Controller")
    print("=====================================")
    
    # Check if Gazebo is running
    try:
        result = subprocess.run(['pgrep', 'gazebo'], capture_output=True, text=True)
        if result.returncode != 0:
            print("⚠️  Warning: Gazebo doesn't appear to be running")
            print("   Start Gazebo first with: gazebo worlds/clean_test.world")
            print("   Then run this controller")
    except FileNotFoundError:
        print("❌ Error: Could not check if Gazebo is running")
    
    controller = GazeboFormationController()
    
    # Show available formations
    print(f"\n📋 Available formations: {', '.join(controller.formation_patterns.keys())}")
    print(f"🎯 Current formation: {controller.current_formation}")
    
    # Start interactive mode
    controller.interactive_control()

if __name__ == "__main__":
    main()
