#!/usr/bin/env python3
"""
Gazebo Formation Flying Controller
Controls 3 drones in Gazebo for formation flying demonstrations
"""

import rospy
import math
import time
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String

class GazeboFormationController:
    def __init__(self):
        rospy.init_node('gazebo_formation_controller', anonymous=True)
        
        # Publishers for each drone
        self.drone1_pub = rospy.Publisher('/drone1/cmd_vel', Twist, queue_size=10)
        self.drone2_pub = rospy.Publisher('/drone2/cmd_vel', Twist, queue_size=10)
        self.drone3_pub = rospy.Publisher('/drone3/cmd_vel', Twist, queue_size=10)
        
        # Subscribers for drone positions
        self.drone1_sub = rospy.Subscriber('/drone1/odom', Odometry, self.drone1_callback)
        self.drone2_sub = rospy.Subscriber('/drone2/odom', Odometry, self.drone2_callback)
        self.drone3_sub = rospy.Subscriber('/drone3/odom', Odometry, self.drone3_callback)
        
        # Current drone positions
        self.drone1_pos = [0, 0, 5]
        self.drone2_pos = [-3, -3, 5]
        self.drone3_pos = [3, -3, 5]
        
        # Formation parameters
        self.formation_spacing = 3.0
        self.formation_height = 5.0
        self.formation_center = [0, 0, 5]
        
        # Control parameters
        self.rate = rospy.Rate(10)  # 10 Hz
        self.kp = 0.5  # Proportional gain
        
        # Formation types
        self.formation_types = {
            'triangle': self.triangle_formation,
            'line': self.line_formation,
            'square': self.square_formation,
            'v_formation': self.v_formation,
            'circle': self.circle_formation
        }
        
        self.current_formation = 'triangle'
        
        # Status publisher
        self.status_pub = rospy.Publisher('/formation/status', String, queue_size=10)
        
        rospy.loginfo("Gazebo Formation Controller initialized")
        
    def drone1_callback(self, msg):
        """Update drone 1 position"""
        self.drone1_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        
    def drone2_callback(self, msg):
        """Update drone 2 position"""
        self.drone2_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        
    def drone3_callback(self, msg):
        """Update drone 3 position"""
        self.drone3_pos = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        
    def triangle_formation(self):
        """Triangle formation positions"""
        center = self.formation_center
        spacing = self.formation_spacing
        
        target1 = [center[0], center[1], center[2]]
        target2 = [center[0] - spacing, center[1] - spacing, center[2]]
        target3 = [center[0] + spacing, center[1] - spacing, center[2]]
        
        return target1, target2, target3
        
    def line_formation(self):
        """Line formation positions"""
        center = self.formation_center
        spacing = self.formation_spacing
        
        target1 = [center[0], center[1], center[2]]
        target2 = [center[0] - spacing, center[1], center[2]]
        target3 = [center[0] + spacing, center[1], center[2]]
        
        return target1, target2, target3
        
    def square_formation(self):
        """Square formation positions"""
        center = self.formation_center
        spacing = self.formation_spacing
        
        target1 = [center[0], center[1], center[2]]
        target2 = [center[0] - spacing, center[1] - spacing, center[2]]
        target3 = [center[0] + spacing, center[1] + spacing, center[2]]
        
        return target1, target2, target3
        
    def v_formation(self):
        """V-formation positions"""
        center = self.formation_center
        spacing = self.formation_spacing
        
        target1 = [center[0], center[1], center[2]]
        target2 = [center[0] - spacing, center[1] - spacing, center[2]]
        target3 = [center[0] + spacing, center[1] - spacing, center[2]]
        
        return target1, target2, target3
        
    def circle_formation(self):
        """Circle formation positions"""
        center = self.formation_center
        radius = self.formation_spacing
        
        target1 = [center[0], center[1], center[2]]
        target2 = [center[0] - radius, center[1], center[2]]
        target3 = [center[0] + radius, center[1], center[2]]
        
        return target1, target2, target3
        
    def change_formation(self, formation_type):
        """Change formation type"""
        if formation_type in self.formation_types:
            self.current_formation = formation_type
            rospy.loginfo(f"Changed formation to: {formation_type}")
            self.status_pub.publish(f"Formation changed to {formation_type}")
        else:
            rospy.logwarn(f"Unknown formation type: {formation_type}")
            
    def move_formation(self, dx, dy, dz):
        """Move entire formation"""
        self.formation_center[0] += dx
        self.formation_center[1] += dy
        self.formation_center[2] += dz
        rospy.loginfo(f"Formation center moved to: {self.formation_center}")
        
    def calculate_velocity(self, current_pos, target_pos):
        """Calculate velocity command to reach target"""
        dx = target_pos[0] - current_pos[0]
        dy = target_pos[1] - current_pos[1]
        dz = target_pos[2] - current_pos[2]
        
        # Apply proportional control
        vx = self.kp * dx
        vy = self.kp * dy
        vz = self.kp * dz
        
        # Limit maximum velocity
        max_vel = 2.0
        vx = max(-max_vel, min(max_vel, vx))
        vy = max(-max_vel, min(max_vel, vy))
        vz = max(-max_vel, min(max_vel, vz))
        
        return vx, vy, vz
        
    def send_velocity_command(self, drone_pub, vx, vy, vz):
        """Send velocity command to a drone"""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.linear.z = vz
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        
        drone_pub.publish(twist)
        
    def run_formation_control(self):
        """Main control loop"""
        rospy.loginfo("Starting formation control loop")
        
        while not rospy.is_shutdown():
            try:
                # Get target positions for current formation
                target1, target2, target3 = self.formation_types[self.current_formation]()
                
                # Calculate velocities for each drone
                v1x, v1y, v1z = self.calculate_velocity(self.drone1_pos, target1)
                v2x, v2y, v2z = self.calculate_velocity(self.drone2_pos, target2)
                v3x, v3y, v3z = self.calculate_velocity(self.drone3_pos, target3)
                
                # Send velocity commands
                self.send_velocity_command(self.drone1_pub, v1x, v1y, v1z)
                self.send_velocity_command(self.drone2_pub, v2x, v2y, v2z)
                self.send_velocity_command(self.drone3_pub, v3x, v3y, v3z)
                
                # Publish status
                status_msg = f"Formation: {self.current_formation}, Center: {self.formation_center}"
                self.status_pub.publish(status_msg)
                
                self.rate.sleep()
                
            except Exception as e:
                rospy.logerr(f"Error in formation control: {e}")
                self.rate.sleep()
                
    def demo_formation_changes(self):
        """Demonstrate different formations"""
        formations = ['triangle', 'line', 'square', 'v_formation', 'circle']
        
        for formation in formations:
            rospy.loginfo(f"Changing to {formation} formation")
            self.change_formation(formation)
            time.sleep(5)  # Hold formation for 5 seconds
            
    def demo_formation_movement(self):
        """Demonstrate formation movement"""
        movements = [
            (2, 0, 0),   # Move right
            (0, 2, 0),   # Move forward
            (-2, 0, 0),  # Move left
            (0, -2, 0),  # Move back
            (0, 0, 1),   # Move up
            (0, 0, -1)   # Move down
        ]
        
        for dx, dy, dz in movements:
            rospy.loginfo(f"Moving formation by ({dx}, {dy}, {dz})")
            self.move_formation(dx, dy, dz)
            time.sleep(3)  # Hold position for 3 seconds

def main():
    try:
        controller = GazeboFormationController()
        
        # Start formation control in a separate thread
        import threading
        control_thread = threading.Thread(target=controller.run_formation_control)
        control_thread.daemon = True
        control_thread.start()
        
        # Wait a bit for initialization
        time.sleep(2)
        
        # Run demonstrations
        rospy.loginfo("Starting formation demonstrations...")
        
        # Demo 1: Formation changes
        rospy.loginfo("Demo 1: Formation changes")
        controller.demo_formation_changes()
        
        # Demo 2: Formation movement
        rospy.loginfo("Demo 2: Formation movement")
        controller.demo_formation_movement()
        
        # Keep running
        rospy.loginfo("Formation control running. Press Ctrl+C to stop.")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("Formation controller stopped")
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")

if __name__ == '__main__':
    main()
