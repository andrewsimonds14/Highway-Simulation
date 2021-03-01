#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Weird profile thing necessary for scan data?
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from simple_pid import PID

# Setting global PID parameters
pid = PID(0.5, 0.01, 0.005, setpoint = 0)

# Define the follow distance for a robot to keep
followDistance = 3.0

class scanNode(Node):
    def __init__(self):
        super().__init__('scan')
        self.scan_subscriber = self.create_subscription(LaserScan, '/scan', self.callback, qos_profile_sensor_data)
        self.velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)

    def publish_PID_speed(self,distance):
        velocity = Twist()
        velocity.linear.x = pid(followDistance - distance) #The three is an arbitrary number indicating the distance to keep from the object in front
        velocity.angular.z = 0.0
        self.velocity_publisher.publish(velocity)

    def publish_default_speed(self):
        velocity = Twist()
        velocity.linear.x = 0.4
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

        # Check for follow distance stuff if something is approaching
        if float(dist_head) < followDistance + 2.0:
            self.publish_PID_speed(float(dist_head))
        else:
            self.publish_default_speed()

 
def main(args = None):
        rclpy.init(args=args)
        node = scanNode()
        rclpy.spin(node)
        rclpy.shutdown()
if __name__ == "__main__":
        main()