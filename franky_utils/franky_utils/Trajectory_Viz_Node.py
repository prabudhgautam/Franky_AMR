#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__("trajectory_drawerNode")

        #parameters for the topic names
        self.declare_parameter("odom_topic", "/franky_controller/odom")

        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

#Subscribe to the odometry topic to receive odometry messages
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)

#Publisher for the path topic to publish the trajectory of the robot based on the odometry data
        self.path_pub = self.create_publisher(Path, "/franky_controller/trajectory", 10)

#Initialize a Path message to store the trajectory of the robot
        self.path_msg = Path()

#odom_callback function that is called whenever a new odometry message is received. 
#It extracts the robot's pose from the odometry message, 
#creates a PoseStamped message, and appends it to the Path message. 
#Finally, it publishes the updated Path message to the path topic.
    def odom_callback(self, msg):
        pose_stamped = PoseStamped()

# Give this pose the timestamp of the odometry measurement
        pose_stamped.header.stamp = msg.header.stamp

# Tell ROS/RViz which coordinate frame the pose belongs to
        pose_stamped.header.frame_id = msg.header.frame_id

# Extract the robot's actual pose from Odometry
        pose_stamped.pose = msg.pose.pose

# Update the Path's own header
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.header.frame_id = msg.header.frame_id
    
# Remember this pose
        self.path_msg.poses.append(pose_stamped)

# Publish the complete trajectory
        self.path_pub.publish(self.path_msg)

def main():
    rclpy.init()
    trajectory_drawerNode = TrajectoryNode()
    rclpy.spin(trajectory_drawerNode)
    trajectory_drawerNode.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()                      
