#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math
import cv_bridge
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time

from simple_pid import PID

pid = PID(0.5, 0.01, 0.005, setpoint = 0)
pid.output_limits = (-0.1,0.1)


class FollowTrackAngleNode(Node):
    def __init__(self):
        super().__init__("FollowTrackAngleNode")
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.handle_camera_data, 10)
        self.velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)
    
    def publish_speed(self,angleError):
        velocity = Twist()
        velocity.linear.x = 0.1
        velocity.angular.z = -pid(angleError)
        print('Current Velocity: ',velocity.angular.z)
        self.velocity_publisher.publish(velocity)


    def handle_camera_data(self,msg):
        global perr, ptime, serr, dt
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #transformation
        height = image.shape[0]
        width = image.shape[1]
        region_of_interest_vertices = [
            (0, height),
            (0, height / 1.9),
            (width, height / 1.9),
            (width, height),
        ]

        mask = np.zeros_like(image)
        match_mask_color = [255, 255, 255]  # <-- This line altered for grayscale.
        cv2.fillPoly(mask, np.int32([region_of_interest_vertices]), match_mask_color)
        cropped_image = cv2.bitwise_and(image, mask)

        ret, thresh_image = cv2.threshold(cropped_image, 130, 145, cv2.THRESH_BINARY)

        cannyed_image = cv2.Canny(thresh_image, 50, 150)

        lines = cv2.HoughLinesP(cannyed_image, 1, np.pi/180, 30, maxLineGap=100)

        left_line_x = []
        left_line_y = []
        right_line_x = []
        right_line_y = []


        for line in lines:
            for x1, y1, x2, y2 in line:
                if (x2 - x1) == 0:
                    slope = (y2 - y1)
                else:
                    slope = (y2 - y1) / (x2 - x1) # <-- Calculating the slope.
                #if math.fabs(slope) < 0.3: # <-- Only consider extreme slope
                #    continue
                if slope <= 0: # <-- If the slope is negative, left group.
                    left_line_x.extend([x1, x2])
                    left_line_y.extend([y1, y2])
                else: # <-- Otherwise, right group.
                    right_line_x.extend([x1, x2])
                    right_line_y.extend([y1, y2])

        #line_image = self.draw_lines(image, lines) # <---- Add this call.

        min_y = int(image.shape[0] * (3/5)) # <-- Just below the horizon
        max_y = int(image.shape[0]) # <-- The bottom of the image
        poly_left = np.poly1d(np.polyfit(
            left_line_y,
            left_line_x,
            deg=1
        ))
        left_x_start = int(poly_left(max_y))
        left_x_end = int(poly_left(min_y))
        poly_right = np.poly1d(np.polyfit(
            right_line_y,
            right_line_x,
            deg=1
        ))
        right_x_start = int(poly_right(max_y))
        right_x_end = int(poly_right(min_y))

        '''
        extrapolated_line_image = self.draw_lines(
            image,
            [[
                [left_x_start, max_y, left_x_end, min_y],
                [right_x_start, max_y, right_x_end, min_y],
            ]],
            thickness=5,
        )
        '''

        imgTl = [0,0]
        imgTr = [width,0]
        imgBr = [width,height]
        imgBl = [0,height]
        img_params = np.float32([imgTl,imgTr,imgBr,imgBl])

            
        tl = [left_x_end, min_y]
        tr = [right_x_end, min_y]
        br = [right_x_start, max_y]
        bl = [left_x_start, max_y]
        corner_points_array = np.float32([tl,tr,br,bl])


        matrix = cv2.getPerspectiveTransform(corner_points_array,img_params)
        #transformed_image = cv2.warpPerspective(extrapolated_line_image,matrix,(width,height))
        

        #Use transformed image for PID calculations and steering
        self.calc_angle_error(tl,tr,bl,br)

        #Resize large images
        def resizeImage(img):
            width = int(img.shape[1] * 0.3)
            height = int(img.shape[0] * 0.3)
            dim = (width, height)
            resized = cv2.resize(img,dim,interpolation = cv2.INTER_AREA)
            return resized

        cv2.imshow("original_image", resizeImage(image)) 
        #cv2.imshow("transformed_image", resizeImage(transformed_image)) 
        cv2.waitKey(2)

    def calc_angle_error(self,tl,tr,bl,br):
        #Calculate the angle between three points twice, treating them as two vectors
        # Left Lane
        dotProduct1 = ((tl[0]-bl[0])*(br[0]-bl[0])) + ((tl[1]-bl[1])*(br[1]-bl[1]))
        scalar1 = abs(math.sqrt(((tl[0]-bl[0])**2 + (tl[1]-bl[1])**2))) * abs(math.sqrt(((br[0]-bl[0])**2 + (br[1]-bl[1])**2)))
        laneAngle1 = math.acos(dotProduct1/scalar1)
        print('Angle1: ', laneAngle1)

        # Right lane
        dotProduct2 = ((tr[0]-br[0])*(bl[0]-br[0])) + ((tr[1]-br[1])*(bl[1]-br[1]))
        scalar2 = abs(math.sqrt(((tr[0]-br[0])**2 + (tr[1]-br[1])**2))) * abs(math.sqrt(((bl[0]-br[0])**2 + (bl[1]-br[1])**2)))
        laneAngle2 = math.acos(dotProduct2/scalar2)
        print('Angle2: ', laneAngle2)

        # If angleDiff positive, left lane angle is greater, meaning we are too close to the right side, and vice versa
        angleDiff = laneAngle1 - laneAngle2
        print('Angle Diff: ', angleDiff)

        # Send difference of angles of left and right lane to PID controller
        self.publish_speed(angleDiff/(math.pi/2))

        

def main(args=None):
    rclpy.init(args=args)
    node = FollowTrackAngleNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

    