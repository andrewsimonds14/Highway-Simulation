#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Weird profile thing necessary for scan data?
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class scanNode(Node):
    def __init__(self):
        super().__init__('scan')
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.callback, qos_profile_sensor_data)
        self.velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)

    def publish_speed(self,speed):
        velocity = Twist()
        velocity.linear.x = speed
        velocity.angular.z = 0.0
        self.velocity_publisher.publish(velocity)

    def callback(self,msg):
        dist_back = format(msg.ranges[180], '.2f')
        print('Distance Back: ', dist_back)
        dist_left = format(msg.ranges[90], '.2f')
        print('Distance Left: ', dist_left)
        dist_right = format(msg.ranges[270], '.2f')
        print('Distance Right: ', dist_right)
        dist_head = format(msg.ranges[0], '.2f')
        print('Distance Front: ', dist_head)

        # Stop if something in front
        # Change to use PID when trying to cruise behind another vehicle instead of stop
        if float(dist_head) < 3.0:
            self.publish_speed(0.0)
        else:
            self.publish_speed(0.4)

 
def main(args = None):
        rclpy.init(args=args)
        node = scanNode()
        rclpy.spin(node)
        rclpy.shutdown()
if __name__ == "__main__":
        main()