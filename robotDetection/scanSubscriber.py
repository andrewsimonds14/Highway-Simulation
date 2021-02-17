#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Weird profile thing necessary for scan data?
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

class scanNode(Node):
    def __init__(self):
        super().__init__('scan')
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.callback, qos_profile_sensor_data)

    def callback(self,msg):
        dist_back = format(msg.ranges[180], '.2f')
        print('Distance Back: ', dist_back)
        dist_left = format(msg.ranges[90], '.2f')
        print('Distance Left: ', dist_left)
        dist_right = format(msg.ranges[270], '.2f')
        print('Distance Right: ', dist_right)
        dist_head = format(msg.ranges[0], '.2f')
        print('Distance Front: ', dist_head)

 
def main(args = None):
        rclpy.init(args=args)
        node = scanNode()
        rclpy.spin(node)
        rclpy.shutdown()
if __name__ == "__main__":
        main()