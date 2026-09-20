#!/usr/bin/env python3
#above line is for linux to know which interpreter to use to run this file- its called a shebang line, if on windows, it is not needed
import math
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

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
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

#Publishers and subscribers for wheel commands, velocity commands, joint states, and odometry data.
        self.wheel_cmd_pub = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10)
        self.vel_sub_ = self.create_subscription(TwistStamped, "franky_controller/cmd_vel", self.velCallback, 10)
        self.joint_state_sub_ = self.create_subscription(JointState, "/joint_states", self.jointCallback, 10)
        self.odom_pub = self.create_publisher(Odometry, "franky_controller/odom", 10)
        self.speed_conversion_ = np.array([
            [self.wheel_radius / 2.0, self.wheel_radius / 2.0],
            [self.wheel_radius / self.wheel_separation,
              -self.wheel_radius / self.wheel_separation]
        ])

#Initializing the odometry message with default values, including frame IDs and orientation.       
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = "odom"
        self.odom_msg.child_frame_id = "base_footprint"

#Normalizing the quaternion which represents the orientation of the robot, 
#which means the sqaured coefficients of the quaternion should sum to 1.0,
#and the default orientation is set to no rotation (identity quaternion).
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

#Initializing TF broadcaster for including transform information between the odometry frame and the robot's base frame in the TF tree, 
#which is essential for robot localization and navigation.
        self.tf_br = TransformBroadcaster(self)
        self.transform_stamped = TransformStamped()
        self.transform_stamped.header.frame_id = "odom"
        self.transform_stamped.child_frame_id = "base_footprint"

         
#VELCALLBACK FUNCTION, Inverse kinematics, converting from robot velocity(V and W) to wheel velocity (fi)
    def velCallback(self, msg):
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) #inverse kinematics , converting from robot velocity(V and W) to wheel velocity (fi)
        wheel_speed_msg = Float64MultiArray()
        wheel_speed_msg.data = [wheel_speed[1,0], wheel_speed[0,0]]
        self.wheel_cmd_pub.publish(wheel_speed_msg)

#JOINTCALLBACK FUNCTION, forward kinematics, converting from wheel velocity (fi) to robot velocity(V and W)
    def jointCallback(self, msg):
        if "wheel_left_joint" not in msg.name or \
           "wheel_right_joint" not in msg.name:
            self.get_logger().warning("Required wheel joints not found")
            return

        left_idx = msg.name.index("wheel_left_joint")
        right_idx = msg.name.index("wheel_right_joint")

        current_time = Time.from_msg(msg.header.stamp)

#To handle the first message received, we need to initialize the previous time and wheel positions.
#If this is the first message, we store the current time and wheel positions and return early. This prevents any calculations from being done with uninitialized values.
        if self.prev_time is None:
            self.prev_time = current_time
            self.left_wheel_prev_pos = msg.position[left_idx]
            self.right_wheel_prev_pos = msg.position[right_idx]
            return
        dt = (current_time - self.prev_time).nanoseconds / S_TO_NS

#To avoid division by zero or negative time differences, we check if dt is less than or equal to zero.
#If it is, we log a warning and return early without performing any calculations.
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

#Calcualting the linear and angular velocities of the robot using the wheel velocities,
#and the robot's physical parameters (wheel radius and separation).
        linear_velocity = self.wheel_radius * (fi_right + fi_left) / 2.0
        angular_velocity = self.wheel_radius/self.wheel_separation * (fi_right - fi_left)

#Calculating the change in position (dx, dy) and orientation (dtheta) of the robot based on the linear and angular velocities and the time difference (dt).
        d_s = self.wheel_radius * (dp_right + dp_left) / 2.0
        d_theta = self.wheel_radius/self.wheel_separation * (dp_right - dp_left)
        self.theta += d_theta
        self.x += d_s * math.cos(self.theta)
        self.y += d_s * math.sin(self.theta)

#Compose and publish the odometry message, which includes the robot's position, orientation, and velocity information in world/odom frame.
        q = quaternion_from_euler(0, 0, self.theta)
        
#JointState's timestamp
        self.odom_msg.header.stamp = current_time.to_msg()
        
#Orientation and position of the robot in the odometry message
        self.odom_msg.pose.pose.orientation.x = q[0]
        self.odom_msg.pose.pose.orientation.y = q[1]
        self.odom_msg.pose.pose.orientation.z = q[2]
        self.odom_msg.pose.pose.orientation.w = q[3]
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y

#Velocity of the robot in the odometry message
        self.odom_msg.twist.twist.linear.x = linear_velocity
        self.odom_msg.twist.twist.angular.z = angular_velocity

#Compose the transform message for broadcasting the robot's position and orientation in the TF tree.
        self.transform_stamped.transform.translation.x = self.x
        self.transform_stamped.transform.translation.y = self.y
        self.transform_stamped.transform.rotation.x = q[0]
        self.transform_stamped.transform.rotation.y = q[1]  
        self.transform_stamped.transform.rotation.z = q[2]
        self.transform_stamped.transform.rotation.w = q[3]
        self.transform_stamped.header.stamp = current_time.to_msg()

#Publishing the odometry message to the "franky_controller/odom" topic,
# which can be used by other nodes for localization, navigation, or visualization purposes.
        self.odom_pub.publish(self.odom_msg)

#Broadcasting TF messages between odom and base_footprint
        self.tf_br.sendTransform(self.transform_stamped)
        

def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()