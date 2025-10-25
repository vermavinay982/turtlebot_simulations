#!/usr/bin/env python3
import rclpy
from rclpy.node import Node 
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("pose_subscriber")
        # msg type, topic name, callback, queue size
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10)

    def pose_callback(self, msg: Pose): # type pose
        self.get_logger().info(f"({str(msg.x)}, {str(msg.y)})")

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node) # to call the callback
    rclpy.shutdown()