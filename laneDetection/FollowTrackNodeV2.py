import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import math
from simple_pid import PID

from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

pid = PID(0.5, 0.01, 0.005, setpoint = 0)
pid.output_limits = (-0.5,0.5)

class FollowTrack(Node):
    def __init__(self):
        super().__init__('follow_track')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 5)
        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)
        
    def prepareImage(self, img):

        '''
        Bird's eye view perspective transform followed by Canny Edge Detection.
        Return both the edge+transformed image and the target lane centre
        '''
        perspective_mat = cv2.getPerspectiveTransform(
        np.array([[0,354],[288,251],[352,251],[640,354]], dtype="float32"), 
        np.array([[200,480],[200,0],[440,0],[440,480]], dtype="float32"))
        transformed_only_img = cv2.warpPerspective(img  , perspective_mat, (640,480))
        img = cv2.cvtColor(transformed_only_img.copy(), cv2.COLOR_BGR2GRAY)
        (thresh, img) = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
        img = cv2.blur(img ,(5,5))
        img  = cv2.Canny(img , int(max(0,0.7*np.median(img ))), int(min(255,1.3*np.median(img ))))
        lane_centre = img.shape[1]/2
        return img, transformed_only_img, lane_centre

    def findLanes(self, img):
        '''
        Hough transfrom followed by sorting of detected lines to find right and left lanes.
        Returns x coordinates from left and right lane respectivley sorted by the corresponding y coordinate in ascending order.
        '''
        _lines = cv2.HoughLinesP(
        img,
        rho=10,
        theta=np.pi / 60,
        threshold=150,
        lines=np.array([]),
        minLineLength=10,
        maxLineGap=15
        )
        lines = []
        if _lines is not None:
            for i in range(len(_lines)):
                lines.append(_lines[i][0])
        lines = np.array(lines)

        left_lines = []
        right_lines = []
        lines_sorted = np.array(sorted(lines, key=lambda x: x[1]))
        left_bound = np.max(lines_sorted[:10,0])
        right_bound = np.min(lines_sorted[:10,0])
        for line in lines_sorted:
            if len(left_lines) == 0 and len(right_lines) == 0:
                if abs(line[0]-left_bound) < abs(line[0]-right_bound):
                    left_lines.append(line)
                else:
                    right_lines.append(line)
            elif len(left_lines) == 0:
                if abs(line[0]-left_bound) < abs(line[0]-right_lines[-1][0]):
                    left_lines.append(line)
                else:
                    right_lines.append(line)
            elif len(right_lines) == 0:
                if abs(line[0]-left_lines[-1][0]) < abs(line[0]-right_bound):
                    left_lines.append(line)
                else:
                    right_lines.append(line)
            else:
                if abs(line[0]-left_lines[-1][0]) < abs(line[0]-right_lines[-1][0]):
                    left_lines.append(line)
                else:
                    right_lines.append(line)

        left_points_x = []
        left_points_y = []
        right_points_x = []
        right_points_y = []
        for i in left_lines:
            left_points_x.extend([i[0], i[2]])
            left_points_y.extend([i[1], i[3]])
        for i in right_lines:
            right_points_x.extend([i[0], i[2]])
            right_points_y.extend([i[1], i[3]])



        poly_left = np.poly1d(np.polyfit(left_points_y, left_points_x, deg=3))

        poly_right = np.poly1d(np.polyfit(right_points_y,right_points_x,deg=3))


        return poly_left, poly_right
    
    
    def showLanes(self, poly_left, poly_right, img):
        max_y = img.shape[0]
        print(max_y)
        y = np.linspace(0, max_y)

        start_centre = img.shape[1] / 2
        start_left = np.polyval(poly_left, max_y)
        start_right = np.polyval(poly_right, max_y)
        avg_start = (start_left+start_right)/2
        error = (avg_start - start_centre) / start_centre


        left_line = np.array(list(zip(np.polyval(poly_left, y),y)), np.int32)
        right_line = np.array(list(zip(np.polyval(poly_right, y),y)), np.int32)

        img = cv2.polylines(img,[left_line], False, [255,0,0], 5)
        img = cv2.polylines(img,[right_line], False, [0,0,255], 5)
        img = cv2.line(img, (int(avg_start), 0), (int(avg_start), max_y), (0,255,0), 5, cv2.LINE_AA)
    
    
    
        return img, error

    def callback(self, ros_msg):
        cv_image = self.bridge.imgmsg_to_cv2(ros_msg,desired_encoding='bgr8')

        processed_img, transformed_img, lane_centre = self.prepareImage(cv_image)

        poly_left, poly_right = self.findLanes(processed_img)

        extrap_img, error = self.showLanes(poly_left, poly_right, transformed_img)
        
        velocity = Twist()
        velocity.linear.x = 0.2
        velocity.angular.z = pid(error)
        self.pub.publish(velocity)
        
        cv2.imshow("Lanes Detected Image", extrap_img)
        cv2.waitKey(2)

            

def main(args=None):
    rclpy.init(args=args)

    follow_track = FollowTrack()

    print("Starting up lane following node")
    rclpy.spin(follow_track)

if __name__ == '__main__':
    main()