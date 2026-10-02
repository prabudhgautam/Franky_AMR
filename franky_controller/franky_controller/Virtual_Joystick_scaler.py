#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from geometry_msgs.msg import Twist
from twist_mux_msgs.action import JoyTurbo


class VirtualJoystickScaler(Node):
    def __init__(self):
        super().__init__('virtual_joystick_scaler')

        # Configuration parameters with explicit float retrieval
        self.declare_parameter("warning_scale", 0.4)  # 40% of max speed in warning zone
        
        self.warning_scale: float = (
            self.get_parameter("warning_scale")
            .get_parameter_value()
            .double_value
        )
        self.current_scale: float = 1.0

        # Velocity Subscriber & Publisher
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.vel_callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

        # Action Servers for Safety Node
        self._decrease_server = ActionServer(
            self,
            JoyTurbo,
            'joy_turbo_decrease',
            self.decrease_speed_callback
        )
        self._increase_server = ActionServer(
            self,
            JoyTurbo,
            'joy_turbo_increase',
            self.increase_speed_callback
        )

        self.get_logger().info('Virtual Joystick Scaler & JoyTurbo Action Servers active.')

    def vel_callback(self, msg: Twist):
        scaled_msg = Twist()
        
        # Expressed explicitly to satisfy Pylance type checking
        scaled_msg.linear.x = float(msg.linear.x) * self.current_scale
        scaled_msg.linear.y = float(msg.linear.y) * self.current_scale
        scaled_msg.linear.z = float(msg.linear.z) * self.current_scale
        
        scaled_msg.angular.x = float(msg.angular.x) * self.current_scale
        scaled_msg.angular.y = float(msg.angular.y) * self.current_scale
        scaled_msg.angular.z = float(msg.angular.z) * self.current_scale

        self.pub.publish(scaled_msg)

    def decrease_speed_callback(self, goal_handle):
        self.current_scale = self.warning_scale
        self.get_logger().warn(f'Speed decreased! Current scale: {self.current_scale * 100:.0f}%')
        
        goal_handle.succeed()
        return JoyTurbo.Result()

    def increase_speed_callback(self, goal_handle):
        self.current_scale = 1.0
        self.get_logger().info('Speed restored! Current scale: 100%')
        
        goal_handle.succeed()
        return JoyTurbo.Result()


def main():
    rclpy.init()
    node = VirtualJoystickScaler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()