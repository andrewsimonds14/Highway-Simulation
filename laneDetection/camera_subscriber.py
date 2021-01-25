#!/usr/bin/env python3

import numpy as np
import cv2
import math
import cv_bridge
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image


class getCameraDataNode(Node):
    def __init__(self):
        super().__init__("getCamDataNode")
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.handle_camera_data, 10)

    def handle_camera_data(self,msg):
        global perr, ptime, serr, dt
        og_image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        #transformation
        #Extract dimensions and create the ROI corners before cropping image to remove horizon line
        height = og_image.shape[0]
        width = og_image.shape[1]

        
        #Calculating Points for Birds-Eye Transform
        imgTl = [0,0]
        imgTr = [width,0]
        imgBr = [width,height]
        imgBl = [0,height]
        img_params = np.float32([imgTl,imgTr,imgBr,imgBl])
        
        
        tl = [0,height/1.8]
        bl = [0, height/1.2]
        tr = [width, height/1.8]
        br = [width, height/1.2]
        corner_points_array = np.float32([tl,tr,br,bl])
        
        #Calculating transformation matrix and applying it to original image
        matrix = cv2.getPerspectiveTransform(corner_points_array,img_params)
        og_image = cv2.warpPerspective(og_image,matrix,(width,height))
        
        
        #Calculate thresholds and apply to Canny Edge Detection
        image = cv2.blur(og_image,(5,5))
        lower = int(max(0,0.7*np.median(image)))
        upper = int(min(255,1.3*np.median(image)))
        image = cv2.Canny(image, lower, upper)
        
        kernel = np.ones((3,3),np.uint8)
        image = cv2.dilate(image,kernel,iterations = 2)
        
        #Calculate Hough Transform to generate lines for each lane
        lines = cv2.HoughLinesP(
            image,
            rho=10,
            theta=np.pi / 60,
            threshold=150,
            lines=np.array([]),
            minLineLength=10,
            maxLineGap=15
        )
        
        
        #Group lines into left and right and rmove any flat ones
        left_line_x = []
        left_line_y = []
        right_line_x = []
        right_line_y = []
        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1) # <-- Calculating the slope.
                if math.fabs(slope) < 0.3: # <-- Only consider extreme slope
                    continue
                if slope <= 0: # <-- If the slope is negative, left group.
                    left_line_x.extend([x1, x2])
                    left_line_y.extend([y1, y2])
                else: # <-- Otherwise, right group.
                    right_line_x.extend([x1, x2])
                    right_line_y.extend([y1, y2])
        
        #Use utility function to draw Hough Tranform lines onto original image
        #line_image = draw_lines(image, lines) # <---- Add this call.
        try:
            poly_left = np.poly1d(np.polyfit(
                left_line_y,
                left_line_x,
                deg=2 #Change to 1 if you want a linear fit
            ))
            
            poly_right = np.poly1d(np.polyfit(
                right_line_y,
                right_line_x,
                deg=2 #Change to 1 if you want a linear fit
            ))
        except:
            print("No lanes detected")
            return
        
        y = np.linspace(image.shape[0]/6, image.shape[0])
        
        #Formatting x and y for use with polylines to display lane overlay
        left_line = np.array(list(zip(np.polyval(poly_left, y),y)), np.int32)
        right_line = np.array(list(zip(np.polyval(poly_right, y),y)), np.int32)
        left_line = left_line.reshape((-1, 1, 2))
        right_line = right_line.reshape((-1, 1, 2)) 
         
        #extrap_image = cv2.polylines(cv2.cvtColor(og_image.copy(),cv2.COLOR_GRAY2RGB),[left_line], False, [255,0,0], 5)
        extrap_image = cv2.polylines(og_image.copy(),[left_line], False, [255,0,0], 5)
        extrap_image = cv2.polylines(extrap_image,[right_line], False, [255,0,0], 5)
        
        #inv_matrix = np.linalg.pinv(matrix)
        #extrap_image = cv2.warpPerspective(extrap_image,inv_matrix,(width,height))

        cv2.imshow("transformed_image", extrap_image)
        cv2.waitKey(2)


def main(args=None):
    rclpy.init(args=args)
    node = getCameraDataNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

    