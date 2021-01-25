#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class locationNode(Node):
    def __init__(self):
        super().__init__('location')
        self.location_subscriber = self.create_subscription(Odometry, '/odom', self.publish_pos, 10)
 
        self.subscription #prevents unused variable warning
 
    
    def publish_pos(msg):
          global x_robot, y_robot
          x_robot = msg.pose.pose.position.x
          y_robot = msg.pose.pose.position.y
          print(x_robot)
          print(y_robot)
 
def main(args = None):
        rclpy.init(args=args)
        node = locationNode()
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
if __name__ == "__main__":
        main()