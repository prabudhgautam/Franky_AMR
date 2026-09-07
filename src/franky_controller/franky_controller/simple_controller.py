#!/usr/bin/env python3
#above line is for linux to know which interpreter to use to run this file- its called a shebang line, if on windows, it is not needed
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped

class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")

        self.declare_parameter("wheel_radius", 0.033)
        self.declare_parameter("wheel_separation", 0.17)

        self.wheel_radius  = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation = self.get_parameter("wheel_separation").get_parameter_value().double_value

        self.get_logger().info(f"using wheel_radius: {self.wheel_radius}")
        self.get_logger().info(f"using wheel_separation: {self.wheel_separation}")

        self.left_wheel_prev_pos = 0.0
        self.right_wheel_prev_pos = 0.0
        self.prev_time = None

        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "franky_controller/cmd_vel", self.velCallback, 10)
        self.joint_state_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.speed_conversion_ = np.array([
            [self.wheel_radius / 2.0, self.wheel_radius / 2.0],
            [self.wheel_radius / self.wheel_separation,
              -self.wheel_radius / self.wheel_separation]
        ])
        
        self.get_logger().info(f"Conversion matrix is {self.speed_conversion_}")


    def velCallback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) #inverse kinematics , converting from robot velocity(V and W) to wheel velocity (fi)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0,0]]
        self.wheel_cmd_pub.publish(wheel_speed_msg)
        self.get_logger().info(f"Published wheel speeds: {wheel_speed_msg.data}")

    def jointCallback(self, msg):
        if "wheel_left_joint" not in msg.name or \
           "wheel_right_joint" not in msg.name:
            self.get_logger().warning("Required wheel joints not found")
            return

        left_idx = msg.name.index("wheel_left_joint")
        right_idx = msg.name.index("wheel_right_joint")

        current_time = Time.from_msg(msg.header.stamp)

        if self.prev_time is None:
            self.prev_time = current_time
            self.left_wheel_prev_pos = msg.position[left_idx]
            self.right_wheel_prev_pos = msg.position[right_idx]
            return
        dt = (current_time - self.prev_time).nanoseconds / S_TO_NS

        if dt <= 0.0:
            self.get_logger().warning("Invalid joint state time difference, skipping message")
            self.prev_time = current_time
            return

        dp_left = msg.position[left_idx] - self.left_wheel_prev_pos
        dp_right = msg.position[right_idx] - self.right_wheel_prev_pos

        self.left_wheel_prev_pos = msg.position[left_idx]
        self.right_wheel_prev_pos = msg.position[right_idx]
        self.prev_time = current_time

        fi_left = dp_left / dt
        fi_right = dp_right / dt

        linear_velocity = self.wheel_radius * (fi_right + fi_left) / 2.0
        angular_velocity = self.wheel_radius/self.wheel_separation * (fi_right - fi_left)

        self.get_logger().info(f"linear_velocity: {linear_velocity}, angular_velocity: {angular_velocity}")

def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()