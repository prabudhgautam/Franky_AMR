import rclpy
from rclpy.node import Node
from std_msgs.msg import String



class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simplepublisher')

        self.pub = self.create_publisher(String, 'chatter', 10)
        self.counter = 0
        self.frequency = 1.0  # Frequency in Hz
        self.get_logger().info(f"Publishing at {self.frequency} Hz")
        self.timer = self.create_timer(self.frequency, self.timerCallback)

    def timerCallback(self):
        msg = String()
        msg.data = f"Hello World: {self.counter}"
        self.pub.publish(msg) 
        self.counter += 1
    


def main():
        rclpy.init()
        simplepublisher = SimplePublisher()
        rclpy.spin(simplepublisher)
        simplepublisher.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()