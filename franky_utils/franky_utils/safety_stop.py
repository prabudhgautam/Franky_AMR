#!/usr/bin/env python3
import rclpy
import time
import math
from enum import Enum 
from rclpy.node import Node
from rclpy.action import ActionClient
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from twist_mux_msgs.action import JoyTurbo
from visualization_msgs.msg import Marker, MarkerArray


class State(Enum):
    FREE = 0
    WARNING = 1
    DANGER = 2


class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop_node")

        self.declare_parameter("danger_distance", 0.35)
        self.declare_parameter("warning_distance", 0.60)        
        self.declare_parameter("scan_topic", "scan")
        self.declare_parameter("safety_stop_topic", "safety_stop")

        self.danger_distance = self.get_parameter("danger_distance").get_parameter_value().double_value
        self.warning_distance = self.get_parameter("warning_distance").get_parameter_value().double_value
        self.scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        self.safety_stop_topic = self.get_parameter("safety_stop_topic").get_parameter_value().string_value

        self.is_first_msg = True
        self.state = State.FREE
        self.prev_state = State.FREE

        self.laser_sub = self.create_subscription(LaserScan, self.scan_topic, self.laser_callback, 10)
        self.safety_stop_pub = self.create_publisher(Bool, self.safety_stop_topic, 10)
        self.zones_pub = self.create_publisher(MarkerArray, 'zones', 10)

        self.decrease_speed_client = ActionClient(self, JoyTurbo, "joy_turbo_decrease")
        self.increase_speed_client = ActionClient(self, JoyTurbo, "joy_turbo_increase")

        while not self.decrease_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_decrease not available waiting..")
            time.sleep(1.0)

        while not self.increase_speed_client.wait_for_server(timeout_sec=1.0) and rclpy.ok():
            self.get_logger().warn("Action /joy_turbo_increase not available waiting..")
            time.sleep(1.0)

        # Store Marker attributes on `self` to avoid index lookup issues in Pylance
        self.warning_zone = Marker()
        self.warning_zone.id = 0
        self.warning_zone.type = Marker.CYLINDER
        self.warning_zone.action = Marker.ADD
        self.warning_zone.scale.z = 0.001
        self.warning_zone.scale.x = self.warning_distance * 2.0
        self.warning_zone.scale.y = self.warning_distance * 2.0
        self.warning_zone.color.r = 1.0
        self.warning_zone.color.g = 0.984
        self.warning_zone.color.b = 0.0
        self.warning_zone.color.a = 0.2

        self.danger_zone = Marker()
        self.danger_zone.id = 1
        self.danger_zone.type = Marker.CYLINDER
        self.danger_zone.action = Marker.ADD
        self.danger_zone.scale.z = 0.001
        self.danger_zone.scale.x = self.danger_distance * 2.0
        self.danger_zone.scale.y = self.danger_distance * 2.0
        self.danger_zone.color.r = 1.0
        self.danger_zone.color.g = 0.0
        self.danger_zone.color.b = 0.0
        self.danger_zone.color.a = 0.2
        self.danger_zone.pose.position.z = 0.01

        self.zones = MarkerArray()
        self.zones.markers = [self.warning_zone, self.danger_zone]

    def laser_callback(self, msg: LaserScan):
        self.state = State.FREE
        
        for range_value in msg.ranges:
            if not math.isinf(range_value) and not math.isnan(range_value) and range_value > msg.range_min:
                if range_value <= self.warning_distance:
                    self.state = State.WARNING

                    if range_value <= self.danger_distance:
                        self.state = State.DANGER
                        break

        # Handle Action Calls ONLY on state transitions
        if self.state != self.prev_state:
            if self.state == State.WARNING:
                self.decrease_speed_client.send_goal_async(JoyTurbo.Goal())
                self.warning_zone.color.a = 1.0
                self.danger_zone.color.a = 0.2
                self.get_logger().info("Entering WARNING Zone: Speed decreased.")

            elif self.state == State.DANGER:
                self.warning_zone.color.a = 1.0
                self.danger_zone.color.a = 1.0
                self.get_logger().warn("Entering DANGER Zone: Emergency Stop activated!")

            elif self.state == State.FREE:
                self.increase_speed_client.send_goal_async(JoyTurbo.Goal())
                self.warning_zone.color.a = 0.2
                self.danger_zone.color.a = 0.2
                self.get_logger().info("Area CLEAR: Full speed restored.")

            self.prev_state = self.state

        # ALWAYS publish current safety_stop state every callback for twist_mux & ROS CLI subscribers
        is_safety_stop = Bool()
        is_safety_stop.data = (self.state == State.DANGER)
        self.safety_stop_pub.publish(is_safety_stop)
        

        if self.is_first_msg:
            self.warning_zone.header.frame_id = msg.header.frame_id
            self.danger_zone.header.frame_id = msg.header.frame_id
            self.is_first_msg = False

        self.zones.markers = [self.warning_zone, self.danger_zone]
        self.zones_pub.publish(self.zones)


def main():
    rclpy.init()
    safety_node = SafetyStop()
    try:
        rclpy.spin(safety_node)
    except KeyboardInterrupt:
        pass
    finally:
        safety_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()