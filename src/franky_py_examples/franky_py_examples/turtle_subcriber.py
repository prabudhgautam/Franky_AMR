import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class Turtlesim(Node):
    def __init__(self):
        super().__init__("turtlesim")

        self.T1_pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.turtle1PoseCallback, 10)
        self.T2_pose_sub = self.create_subscription(Pose, "/turtleninja/pose", self.turtle2PoseCallback, 10)

        self.lastPose_T1 = Pose()
        self.lastPose_T2 = Pose()        

    def turtle1PoseCallback(self, msg):
        self.lastPose_T1 = msg

    def turtle2PoseCallback(self, msg):
        self.lastPose_T2 = msg

        Tx = self.lastPose_T2.x - self.lastPose_T1.x
        Ty = self.lastPose_T2.y - self.lastPose_T1.y

        self.get_logger().info(
            f"translation vector turtle1 --> turtle2\n"
            f"Tx: {Tx}\n"
            f"Ty: {Ty}"
        )
def main():
    rclpy.init()
    turtlesim = Turtlesim()
    rclpy.spin(turtlesim)
    turtlesim.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()