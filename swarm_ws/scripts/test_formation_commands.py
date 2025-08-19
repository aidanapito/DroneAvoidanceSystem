#!/usr/bin/env python3
"""
Formation Flying Command Test Script

This script tests the formation flying controller by:
- Sending formation change commands
- Monitoring formation status
- Testing different formation patterns
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from std_msgs.msg import String
import time


class FormationTestNode(Node):
    """Test node for formation flying commands."""
    
    def __init__(self):
        super().__init__('formation_test_node')
        
        # Initialize parameters
        self.declare_parameter('test_interval', 5.0)
        self.declare_parameter('enable_auto_test', True)
        
        self.test_interval = self.get_parameter('test_interval').value
        self.enable_auto_test = self.get_parameter('enable_auto_test').value
        
        # Initialize services and subscribers
        self.change_formation_client = self.create_client(
            Trigger, '/formation/change'
        )
        
        self.formation_status_sub = self.create_subscription(
            String, '/swarm/formation_status',
            self._formation_status_callback, 10
        )
        
        # Test state
        self.test_start_time = time.time()
        self.formation_count = 0
        self.last_formation_change = 0
        
        # Start test timer
        self.test_timer = self.create_timer(
            self.test_interval, 
            self._test_timer_callback
        )
        
        self.get_logger().info("Formation Test Node initialized")
        self.get_logger().info(f"Auto test: {self.enable_auto_test}")
        self.get_logger().info(f"Test interval: {self.test_interval}s")
    
    def _formation_status_callback(self, msg: String):
        """Handle formation status updates."""
        self.get_logger().info(f"📊 Formation Status: {msg.data}")
        
        # Check if formation is stable
        if "Stable: True" in msg.data:
            self.get_logger().info("✅ Formation is stable!")
        else:
            self.get_logger().info("⏳ Formation is adjusting...")
    
    def _test_timer_callback(self):
        """Timer callback for automatic testing."""
        if not self.enable_auto_test:
            return
        
        current_time = time.time()
        if current_time - self.last_formation_change >= self.test_interval:
            self._test_formation_change()
            self.last_formation_change = current_time
    
    def _test_formation_change(self):
        """Test formation change command."""
        if not self.change_formation_client.service_is_ready():
            self.get_logger().warn("Formation change service not ready")
            return
        
        self.get_logger().info("🔄 Testing formation change...")
        
        # Create request
        request = Trigger.Request()
        
        # Send request asynchronously
        future = self.change_formation_client.call_async(request)
        future.add_done_callback(self._formation_change_callback)
    
    def _formation_change_callback(self, future):
        """Callback for formation change service."""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ Formation change successful: {response.message}")
                self.formation_count += 1
            else:
                self.get_logger().error(f"❌ Formation change failed: {response.message}")
        except Exception as e:
            self.get_logger().error(f"❌ Formation change error: {e}")
    
    def manual_formation_change(self):
        """Manually trigger a formation change."""
        self._test_formation_change()
    
    def get_test_stats(self):
        """Get test statistics."""
        elapsed_time = time.time() - self.test_start_time
        return {
            'elapsed_time': elapsed_time,
            'formation_changes': self.formation_count,
            'changes_per_minute': (self.formation_count / elapsed_time) * 60 if elapsed_time > 0 else 0
        }


def main(args=None):
    """Main function for the formation test node."""
    rclpy.init(args=args)
    
    try:
        test_node = FormationTestNode()
        
        # Manual test mode
        if not test_node.enable_auto_test:
            print("\n🧪 Manual Formation Test Mode")
            print("=" * 40)
            print("Commands:")
            print("  'change' - Change formation")
            print("  'stats'  - Show test statistics")
            print("  'quit'   - Exit")
            print()
            
            while rclpy.ok():
                try:
                    command = input("Enter command: ").strip().lower()
                    
                    if command == 'change':
                        test_node.manual_formation_change()
                    elif command == 'stats':
                        stats = test_node.get_test_stats()
                        print(f"📊 Test Statistics:")
                        print(f"   Elapsed time: {stats['elapsed_time']:.1f}s")
                        print(f"   Formation changes: {stats['formation_changes']}")
                        print(f"   Changes per minute: {stats['changes_per_minute']:.1f}")
                    elif command == 'quit':
                        break
                    else:
                        print("Unknown command. Use: change, stats, or quit")
                        
                except EOFError:
                    break
        else:
            # Auto test mode
            print("\n🤖 Auto Formation Test Mode")
            print("=" * 40)
            print("Automatically testing formation changes every", test_node.test_interval, "seconds")
            print("Press Ctrl+C to stop")
            print()
            
            rclpy.spin(test_node)
            
    except KeyboardInterrupt:
        print("\n⏹️  Test stopped by user")
    except Exception as e:
        print(f"❌ Test error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'test_node' in locals():
            # Show final statistics
            stats = test_node.get_test_stats()
            print(f"\n📊 Final Test Statistics:")
            print(f"   Total time: {stats['elapsed_time']:.1f}s")
            print(f"   Total formation changes: {stats['formation_changes']}")
            print(f"   Average changes per minute: {stats['changes_per_minute']:.1f}")
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()
