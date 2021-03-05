#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class moveTBNode(Node):
    def __init__(self):
        super().__init__("moveTBNode")
        self.velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.velocity_timer_ = self.create_timer( 2.0, self.publish_speed)

    def publish_speed(self):
        velocity = Twist()
        velocity.linear.x = 0.1
        velocity.angular.z = 0.15
        self.velocity_publisher.publish(velocity)
    

def main(args=None):
    rclpy.init(args=args)
    node = moveTBNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()
    