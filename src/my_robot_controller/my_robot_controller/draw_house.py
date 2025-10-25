#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawHouseNode(Node):
    def __init__(self):
        super().__init__("draw_house")
        # message type, topic name, queue size for buffer, 
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.send_straight()
        self.send_rotate()
        self.send_straight()
        self.send_rotate()

        self.timer_ = self.create_timer(0.5, self.send_straight)
        self.timer_ = self.create_timer(2.0, self.send_rotate)

        self.get_logger().info("Draw house node has been started")

    def send_straight(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)
    
    def send_rotate(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 5.0
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrawHouseNode()
    rclpy.spin(node)
    rclpy.shutdown()