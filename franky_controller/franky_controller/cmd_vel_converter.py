#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class CmdVelConverter(Node):
    def __init__(self):
        super().__init__('cmd_vel_converter') #Twist Relay

        # Subscribes to twist_mux output (Twist)
        self.sub = self.create_subscription(Twist, 'franky_controller/cmd_vel_unstamped', self.cb, 10)

        # Publishes TwistStamped required by franky_controller
        self.pub = self.create_publisher(TwistStamped, '/franky_controller/cmd_vel', 10)
        self.get_logger().info('CmdVelConverter active: /franky_controller/cmd_vel_unstamped (Twist) -> /franky_controller/cmd_vel (TwistStamped)')
        
    def cb(self, msg):
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'base_link'
        stamped.twist = msg
        self.pub.publish(stamped)

def main():
    rclpy.init()
    node = CmdVelConverter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
