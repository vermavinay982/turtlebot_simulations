#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from  turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial

"""
ros2 run turtlesim turtlesim_node 
ros2 run my_robot_controller turtle_controller 
ros2 topic hz /turtle1/pose - topic hertz

"""
"""
0,0 is bottom left, 
11,11 is top right, 
5,5 is center
"""
class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.previous_x = 0
        self.cmd_vel_pub_ = self.create_publisher(
            Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(
            Pose, "/turtle1/pose", self.pose_callback, 10
        )
        self.get_logger().info("Turtle Controller Started")

    def pose_callback(self, pose: Pose, ):
        pass
        cmd = Twist()

        if (pose.x>8 or pose.y>8) or (pose.x<2 or pose.y<2):
            cmd.angular.z = 5.0
            cmd.linear.x = 5.0
            self.get_logger().info(f"Rotating")
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        
        if pose.x>5.5 and self.previous_x <= 5.5:
            self.previous_x = pose.x
            self.get_logger().info(f"to Red")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif pose.x<=5.5 and self.previous_x > 5.5:
            self.previous_x = pose.x
            self.get_logger().info(f"to Green")
            self.call_set_pen_service(0, 255, 0, 3, 0)

        self.cmd_vel_pub_.publish(cmd)
        self.get_logger().info(f"({str(pose.x)}, {str(pose.y)})")

    def call_set_pen_service(self, r, g, b, width, off):
        # ros2 service type /turtle1/set_pen -> turtlesim/srv/SetPen
        # ros2 interface show  turtlesim/srv/SetPen -> uint8 r,g,b,width,off
        # service type, service name, 
        client = self.create_client(SetPen, "/turtle1/set_pen") # obj to represent the client, 
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for service...")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        # need async, not to block the thread to wait for it to change
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            response = future.result() # there is nothing in response for this service
        except Exception as e: 
            self.get_logger().error("Service call failed: %r" % (e, ))

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()