#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("first_node")
        self.counter_ = 0
        self.create_timer(1.0, self.timer_callback) # create a loop function,
        self.create_timer(2.0, self.timer_callback2) # create a loop function,

        # self.get_logger().info("hello from Ros2")

    def timer_callback(self): # call this function or any function
        self.get_logger().info(f"Hello [{str(self.counter_)}]")
        self.counter_ += 1

    def timer_callback2(self): # call this function or any function
        self.get_logger().info(f"Greetings: [{str(self.counter_)}]")
        self.counter_ += 1


def main(args=None):
    rclpy.init(args=args)
    node = MyNode() # just a instance of class. 
    rclpy.spin(node) # create a while loop here
    rclpy.shutdown()
    pass

if __name__=='__main__':
    main()

