import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class subscriber(Node):
    def __init__(self):
        super().__init__('stringsubscriber')
        self.sub = self.create_subscription(String, 'chatter', self.subCallback, 10)


    def subCallback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")


def main():
    rclpy.init()
    stringsubscriber = subscriber()
    rclpy.spin(stringsubscriber)
    stringsubscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()