import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'diff_cont/odom',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        print("Listener active")

        self.x = 0
        self.y = 0
        self.angz = 0
        self.angw = 0

    def listener_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.angz = msg.pose.pose.orientation.z
        self.angw = msg.pose.pose.orientation.w