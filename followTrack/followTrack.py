#!/usr/bin/env python3
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import cv2
import math
import cv_bridge
import rclpy 
import PID
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import time

from simple_pid import PID

pid = PID(0.5, 0.01, 0.005, setpoint = 0)
pid.output_limits = (-0.5,0.5)


class followTrackNode(Node):
    def __init__(self):
        super().__init__("followTrackNode")
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.handle_camera_data, 10)
        self.velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)
    
    def publish_speed(self,pixelDifference):
        print(pixelDifference)
        velocity = Twist()
        velocity.linear.x = 0.3
        velocity.angular.z = pid(pixelDifference/100000)
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

        extrapolated_line_image = self.draw_lines(
            image,
            [[
                [left_x_start, max_y, left_x_end, min_y],
                [right_x_start, max_y, right_x_end, min_y],
            ]],
            thickness=5,
        )

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
        print(corner_points_array)
        time.sleep(2)

        matrix = cv2.getPerspectiveTransform(corner_points_array,img_params)
        transformed_image = cv2.warpPerspective(extrapolated_line_image,matrix,(width,height))
        
        #Uncomment below for updated lane detection
        
        # og_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # #Extract dimensions and create the ROI corners before cropping image to remove horizon line
        # height = og_image.shape[0]
        # width = og_image.shape[1]

        
        # #Calculating Points for Birds-Eye Transform
        # imgTl = [0,0]
        # imgTr = [width,0]
        # imgBr = [width,height]
        # imgBl = [0,height]
        # img_params = np.float32([imgTl,imgTr,imgBr,imgBl])
        
        
        # tl = [0,height/1.8]
        # bl = [0, height/1.2]
        # tr = [width, height/1.8]
        # br = [width, height/1.2]
        # corner_points_array = np.float32([tl,tr,br,bl])
        
        # #Calculating transformation matrix and applying it to original image
        # matrix = cv2.getPerspectiveTransform(corner_points_array,img_params)
        # og_image = cv2.warpPerspective(og_image,matrix,(width,height))
        
        
        # #Calculate thresholds and apply to Canny Edge Detection
        # image = cv2.blur(og_image,(5,5))
        # lower = int(max(0,0.7*np.median(image)))
        # upper = int(min(255,1.3*np.median(image)))
        # image = cv2.Canny(image, lower, upper)
        
        # kernel = np.ones((3,3),np.uint8)
        # image = cv2.dilate(image,kernel,iterations = 2)
        
        # #Calculate Hough Transform to generate lines for each lane
        # lines = cv2.HoughLinesP(
            # image,
            # rho=10,
            # theta=np.pi / 60,
            # threshold=150,
            # lines=np.array([]),
            # minLineLength=10,
            # maxLineGap=15
        # )
        
        
        # #Group lines into left and right and rmove any flat ones
        # left_line_x = []
        # left_line_y = []
        # right_line_x = []
        # right_line_y = []
        # for line in lines:
            # for x1, y1, x2, y2 in line:
                # slope = (y2 - y1) / (x2 - x1) # <-- Calculating the slope.
                # if math.fabs(slope) < 0.3: # <-- Only consider extreme slope
                    # continue
                # if slope <= 0: # <-- If the slope is negative, left group.
                    # left_line_x.extend([x1, x2])
                    # left_line_y.extend([y1, y2])
                # else: # <-- Otherwise, right group.
                    # right_line_x.extend([x1, x2])
                    # right_line_y.extend([y1, y2])
        
        # #Use utility function to draw Hough Tranform lines onto original image
        # #line_image = draw_lines(image, lines) # <---- Add this call.
        # try:
            # poly_left = np.poly1d(np.polyfit(
                # left_line_y,
                # left_line_x,
                # deg=2 #Change to 1 if you want a linear fit
            # ))
            
            # poly_right = np.poly1d(np.polyfit(
                # right_line_y,
                # right_line_x,
                # deg=2 #Change to 1 if you want a linear fit
            # ))
        # except:
            # print("No lanes detected")
            # return
        
        # y = np.linspace(image.shape[0]/6, image.shape[0])
        
        # #Formatting x and y for use with polylines to display lane overlay
        # left_line = np.array(list(zip(np.polyval(poly_left, y),y)), np.int32)
        # right_line = np.array(list(zip(np.polyval(poly_right, y),y)), np.int32)
        # left_line = left_line.reshape((-1, 1, 2))
        # right_line = right_line.reshape((-1, 1, 2)) 
         
        # #extrap_image = cv2.polylines(cv2.cvtColor(og_image.copy(),cv2.COLOR_GRAY2RGB),[left_line], False, [255,0,0], 5)
        # extrap_image = cv2.polylines(og_image.copy(),[left_line], False, [255,0,0], 5)
        # extrap_image = cv2.polylines(extrap_image,[right_line], False, [255,0,0], 5)
        # cv2.imshow("transformed_image", extrap_image)
        # cv2.waitKey(2)
        
        #Uncomment above for updated lane detection

        #Use transformed image for PID calculations and steering
        self.steer_robot(transformed_image)

        #Resize large images
        def resizeImage(img):
            width = int(img.shape[1] * 0.3)
            height = int(img.shape[0] * 0.3)
            dim = (width, height)
            resized = cv2.resize(img,dim,interpolation = cv2.INTER_AREA)
            return resized

        cv2.imshow("original_image", resizeImage(image)) 
        cv2.imshow("transformed_image", resizeImage(transformed_image)) 
        cv2.waitKey(2)

    def draw_lines(self, img, lines, color=[255, 0, 0], thickness=3):
        # If there are no lines to draw, exit.
        if lines is None:
            return
        # Make a copy of the original image.
        img = np.copy(img)
        # Create a blank image that matches the original in size.
        line_img = np.zeros(
            (
                img.shape[0],
                img.shape[1],
                3
            ),
            dtype=np.uint8,
        )
        # Loop over all lines and draw them on the blank image.
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
        # Merge the image with the lines onto the original.
        img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
        # Return the modified image.
        return img

    def steer_robot(self,img):
        #Determine how centered the robot is based of the img passed in

        #Split image in two, then determine which side has more black pixels and by how much
        height, width, x = img.shape
        width_cutoff = width // 2
        leftImage = img[:,:width_cutoff]
        rightImage = img[:,width_cutoff:]

        leftBlackPixelCount = (leftImage.shape[0] * leftImage.shape[1]) - cv2.countNonZero(cv2.cvtColor(leftImage, cv2.COLOR_BGR2GRAY))
        rightBlackPixelCount = (rightImage.shape[0] * rightImage.shape[1]) - cv2.countNonZero(cv2.cvtColor(rightImage, cv2.COLOR_BGR2GRAY))

        pixelDifference = rightBlackPixelCount - leftBlackPixelCount

        #Send pixelDifference to speed publisher
        self.publish_speed(pixelDifference)

        

def main(args=None):
    rclpy.init(args=args)
    node = followTrackNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

    