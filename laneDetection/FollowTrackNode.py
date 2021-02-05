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
        img   = cv2.warpPerspective(img  , perspective_mat, (640,480))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (thresh, img) = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
        img = cv2.blur(img ,(5,5))
        img  = cv2.Canny(img , int(max(0,0.7*np.median(img ))), int(min(255,1.3*np.median(img ))))
        lane_centre = img.shape[1]/2
        return img, lane_centre

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
        left_points = []
        right_points = []
        for i in left_lines:
            left_points.append([i[0], i[1]])
            left_points.append([i[2], i[3]])
        for i in right_lines:
            right_points.append([i[0], i[1]])
            right_points.append([i[2], i[3]])
        left_points_sorted = sorted(left_points, key=lambda x: x[1])
        right_points_sorted = sorted(right_points, key=lambda x: x[1])
        return left_points_sorted, right_points_sorted

    def calcPositionError(self, left_points_sorted, right_points_sorted, lane_centre):
        '''
        Calculates normalized error between two detected lanes and centre to feed to a PID controller.
        '''
        if len(left_points_sorted) > 0 and len(right_points_sorted) > 0:
            x_left = left_points_sorted[-1][0]
            x_right = right_points_sorted[-1][0]
            avg_x = (x_left + x_right) / 2
            error = (avg_x - lane_centre) / lane_centre
        else:
            print("No lanes detected")
            error = 0
        return error

    def callback(self, ros_msg):
        cv_image = self.bridge.imgmsg_to_cv2(ros_msg,desired_encoding='bgr8')
        
        processed_img, lane_centre = self.prepareImage(cv_image)

        left_points_sorted, right_points_sorted = self.findLanes(processed_img)

        error = self.calcPositionError(left_points_sorted, right_points_sorted, lane_centre)
        
        
        velocity = Twist()
        velocity.linear.x = 0.2
        velocity.angular.z = pid(error)
        self.pub.publish(velocity)

            

def main(args=None):
    rclpy.init(args=args)

    follow_track = FollowTrack()

    print("Starting up lane following node")
    rclpy.spin(follow_track)

if __name__ == '__main__':
    main()