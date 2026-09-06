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
        self.prev_time = self.get_clock().now()

        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "franky_controller/cmd_vel", self.velCallback, 10)
        self.joint_state_sub_ = self.create_subscription(JointState, "joint_states", self.jointCallback, 10)
        self.speed_conversion_ = np.array([[self.wheel_radius/2, self.wheel_radius/2],
                                           [self.wheel_radius/self.wheel_separation, -self.wheel_radius/self.wheel_separation]])
        
        self.get_logger().info(f"Conversion matrix is {self.speed_conversion_}")


    def velCallback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) #inverse kinematics , converting from robot velocity(V and W) to wheel velocity (fi)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0,0]]
        self.wheel_cmd_pub.publish(wheel_speed_msg)

    def jointCallback(self, msg):
        dp_left = msg.position[1] - self.left_wheel_prev_pos
        dp_right = msg.position[0] - self.right_wheel_prev_pos
        dt = Time.from_msg(msg.header.stamp) - self.prev_time

        self.left_wheel_prev_pos = msg.position[1]
        self.right_wheel_prev_pos = msg.position[0]
        self.prev_time = Time.from_msg(msg.header.stamp)

        fi_left = dp_left/(dt.nanoseconds / S_TO_NS)
        fi_right = dp_right/(dt.nanoseconds / S_TO_NS)

        linear_velocity = self.wheel_radius/2 * (fi_left + fi_right)
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