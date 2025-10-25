#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")
        # message type, topic name, queue size for buffer, 
        self.cmd_vel_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 1)
        self.timer_ = self.create_timer(0.5, self.send_vel_command)
        self.timer_ = self.create_timer(2.0, self.send_vel_command_fwd)

        self.get_logger().info("Draw circle node has been started")

    def send_vel_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 4.0
        self.cmd_vel_pub.publish(msg)
    
    def send_vel_command_fwd(self):
        msg = Twist()
        msg.linear.x = 5.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()