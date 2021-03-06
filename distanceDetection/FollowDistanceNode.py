#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# Weird profile thing necessary for scan data?
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from simple_pid import PID
import time
import numpy as np
import cv2
import keras
from keras.models import load_model
from keras.preprocessing import image
import cv_bridge
from sensor_msgs.msg import Image

kerasModelPath = input('Please enter the filePath for the h5 file (Example: /home/brock/Documents/TurtleBotClassifer.h5): ')

# Setting global PID parameters
pid = PID(0.5, 0.01, 0.005, setpoint = 0)

# Figure out what robot to use
multipleBots = input('More than one robot? (y/n): ')
if multipleBots == 'y':
    robotName = '/robot' + str(input('Enter 1 or 2 to determine which robot to use: '))
else:
    robotName = ''


# Define the follow distance for a robot to keep
followDistance = 2.5

class FollowDistanceNode(Node):
    def __init__(self):
        super().__init__('followDistance')
        self.scan_subscriber = self.create_subscription(LaserScan, '{}/scan'.format(robotName), self.callback, qos_profile_sensor_data)
        self.velocity_publisher = self.create_publisher(Twist, "{}/cmd_vel".format(robotName), 10)
        self.update_time = time.time()
        self.leader_speed_state = 0
        self.leader_speed = 0.4
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image, "{}/camera/image_raw".format(robotName), self.handle_camera_data, 10)
        self.model = keras.models.load_model(kerasModelPath)

    def handle_camera_data(self,msg):
        # global perr, ptime, serr, dt
        bot_img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
        bot_img = cv2.resize(bot_img,(640,480))
        img = np.expand_dims(bot_img,axis=0)
        prediction = int(self.model.predict(img))
        if prediction == 0:
            print('TurtleBot Detected')
        if prediction != 0:
            print('No TurtleBot Detected')

    def publish_PID_speed(self,distance):
        velocity = Twist()
        velocity.linear.x = pid(followDistance - distance)
        print('Current Linear Velocity: ',velocity.linear.x)
        #velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.velocity_publisher.publish(velocity)

    def publish_default_speed(self):
        velocity = Twist()
        if robotName == '/robot1':
            # check if ten secs has elapsed
            if time.time() - self.update_time > 10:
                self.update_time = time.time()
                if self.leader_speed == 0.5:
                    self.leader_speed = 0.2
                else:
                    self.leader_speed = 0.5
            velocity.linear.x = self.leader_speed
        else:
            velocity.linear.x = 0.4
        print('Current Linear Velocity: ',velocity.linear.x)
        #velocity.linear.x = 0.0
        velocity.angular.z = 0.0
        self.velocity_publisher.publish(velocity)

    def callback(self,msg):
        # Check for things in front
        angle = 0
        pidActive = False
        while angle > 330 or angle < 30:
            if msg.ranges[angle] < followDistance + 2.0 and msg.ranges[angle] > 1: # Trying to get rid of faulty laserScan's effect with the lower bound
                print('Distance Front: ',msg.ranges[angle])
                self.publish_PID_speed(float(msg.ranges[angle]))
                pidActive = True
                break
            else:
                if angle == 0:
                    angle = angle + 1
                elif angle < 30:
                    angle = 360 + angle * (-1)
                else:
                    angle = (360 - angle) + 1
        
        if not pidActive:
            self.publish_default_speed()            

        '''
        # Check for follow distance stuff if something is approaching
        if float(dist_front) < followDistance + 2.0:
            self.publish_PID_speed(float(dist_front))
        else:
            self.publish_default_speed()
        '''
 
def main(args = None):
        rclpy.init(args=args)
        node = FollowDistanceNode()
        rclpy.spin(node)
        rclpy.shutdown()
if __name__ == "__main__":
        main()