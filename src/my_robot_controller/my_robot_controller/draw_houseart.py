#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, SetPen
from std_srvs.srv import Empty
import time
 
class HouseArtist(Node):
    def __init__(self):
        super().__init__("house_artist")
        self.heading = 0.0
        self.velocity_pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.loop_rate = self.create_rate(50)
        self.teleport_client = self.create_client(TeleportAbsolute, "/turtle1/teleport_absolute")
        self.pen_client = self.create_client(SetPen, "/turtle1/set_pen")
        self.clear_client = self.create_client(Empty, "/reset")
        for client, label in [
            (self.teleport_client, "teleport_absolute"),
            (self.pen_client, "set_pen"),
            (self.clear_client, "reset"),
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Waiting for {label} service...")
 
    # ---- Movement Primitives ----
    def forward(self, dist, lin_speed=1.0):
        msg = Twist()
        msg.linear.x = lin_speed
        t_end = time.time() + dist / lin_speed
        while time.time() < t_end:
            self.velocity_pub.publish(msg)
            time.sleep(0.02)
        msg.linear.x = 0.0
        self.velocity_pub.publish(msg)
 
    def turn(self, deg, ang_speed=1.0):
        msg = Twist()
        rad = math.radians(deg)
        msg.angular.z = ang_speed if rad > 0 else -ang_speed
        t_end = time.time() + abs(rad) / ang_speed
        while time.time() < t_end:
            self.velocity_pub.publish(msg)
            time.sleep(0.02)
        msg.angular.z = 0.0
        self.velocity_pub.publish(msg)
        self.heading += rad
 
    def pen_control(self, enable, r=255, g=255, b=255, width=2):
        req = SetPen.Request()
        if enable:
            req.r, req.g, req.b, req.width, req.off = r, g, b, width, 0
        else:
            req.r, req.g, req.b, req.width, req.off = 0, 0, 0, 0, 1
        fut = self.pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done():
            self.get_logger().warn("Pen service timeout")
 
    def jump_to(self, x, y):
        self.pen_control(False)
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = float(x), float(y), 0.0
        fut = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done():
            self.get_logger().warn("Teleport timeout")
            return
        self.heading = 0.0
        self.pen_control(True)
 
    def teleport_to(self, x, y, theta=0.0):
        self.pen_control(False)
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = float(x), float(y), float(theta)
        fut = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        if not fut.done():
            self.get_logger().warn("Teleport timeout")
            return
        self.heading = theta
        self.pen_control(True)
 
    # ---- Shape Drawing ----
    def rectangle(self, x, y, w, h):
        self.jump_to(x, y)
        for _ in range(2):
            self.forward(w)
            self.turn(90)
            self.forward(h)
            self.turn(90)
 
    def triangle(self, x, y, base, height):
        self.jump_to(x, y)
        apex_x = x + base / 2
        apex_y = y + height
        angle1 = math.atan2(apex_y - y, apex_x - x)
        dist1 = math.hypot(apex_x - x, apex_y - y)
        self.turn(math.degrees(angle1 - self.heading))
        self.forward(dist1)
        right_x = x + base
        right_y = y
        angle2 = math.atan2(right_y - apex_y, right_x - apex_x)
        dist2 = math.hypot(right_x - apex_x, right_y - apex_y)
        self.turn(math.degrees(angle2 - (self.heading + angle1)))
        self.forward(dist2)
 
    def draw_home(self, x, y):
        base_w = 5.0
        base_h = 4.5
        self.rectangle(x, y, base_w, base_h)
        roof_w = 5.0
        roof_h = 4.0
        self.triangle(x, y + base_h, roof_w, roof_h)
        win_size = 1.0
        win_x = x + 0.5
        win_y = y + 2.0
        self.rectangle(win_x, win_y, win_size, win_size)
        door_w = 1.0
        door_h = 2.5
        door_x = x + base_w - 1.0 - door_w
        self.rectangle(door_x, y, door_w, door_h)
 
    def clear_canvas(self):
        req = Empty.Request()
        fut = self.clear_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut)
        time.sleep(0.5)
 
def main(args=None):
    rclpy.init(args=args)
    artist = HouseArtist()

    x, y = 0, 0
    artist.get_logger().info(f"Placing house at ({x}, {y})...")
    artist.draw_home(x, y)
    input("🏠 House complete. Press Enter to continue...")
    artist.clear_canvas()
    artist.destroy_node()
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()