#ros2 launch turtlebot3_gazebo turtlebot3_road.launch.py
import numpy as np
import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import math
from simple_pid import PID

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

pid = PID(3, 0.1, 0.5, setpoint = 0)

class FollowTrack(Node):
    def __init__(self):
        super().__init__('follow_track')

        self.bridge = CvBridge()

        self.velocity_sub = self.create_subscription(Twist, '/cmd_vel', self.get_speed, 5) 
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.callback, 5)
        
        self.pub = self.create_publisher(Twist, 'cmd_vel', 1)

        self.linearVel = 0.0

    def get_speed(self,msg):
        self.linearVel = msg.linear.x
        
    def prepareImage(self, img):
        '''
        Bird's eye view perspective transform followed by Canny Edge Detection.
        Return both the edge+transformed image and the transformed original image
        '''
        perspective_mat = cv2.getPerspectiveTransform(
        np.array([[0,354],[288,251],[352,251],[640,354]], dtype="float32"), 
        np.array([[200,480],[200,0],[440,0],[440,480]], dtype="float32"))
        transform_only_img   = cv2.warpPerspective(img  , perspective_mat, (640,480))
        img   = cv2.warpPerspective(img  , perspective_mat, (640,480))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (thresh, img) = cv2.threshold(img, 127, 255, cv2.THRESH_BINARY)
        img = cv2.blur(img ,(5,5))
        img  = cv2.Canny(img , int(max(0,0.7*np.median(img ))), int(min(255,1.3*np.median(img ))))
        return img, transform_only_img 

    def findLanes(self, img):
        '''
        Hough transfrom followed by sorting of detected lines to find right and left lanes.
        Returns x coordinates from left and right lane respectivley sorted by the corresponding y coordinate in ascending order.
        '''
        _lines = cv2.HoughLinesP(
        img,
        rho=10,
        theta=np.pi / 60,
        threshold=10,
        lines=np.array([]),
        minLineLength=0,
        maxLineGap=0
        )
        lines = []
        if _lines is not None:
            for i in range(len(_lines)):
                lines.append(_lines[i][0])
        lines = np.array(lines)

        left_lines = []
        right_lines = []
        lines_sorted = np.array(sorted(lines, key=lambda x: x[1]))
        left_bound = np.max(lines_sorted[:20,0])
        right_bound = np.min(lines_sorted[:20,0])
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

    def calcPositionError(self, left_lines, right_lines,img):
        if len(left_lines) > 0 and len(right_lines) > 0:
            x_left = left_lines[-1][0]
            x_right = right_lines[-1][0]
            avg_x = (x_left + x_right) / 2
            lane_centre = img.shape[1]/2
            x_error = (avg_x - lane_centre) / lane_centre
            error = pid(x_error)
            cv2.line(img, (int(x_left), 280), (int(x_left), 480), (255,0,0), 3, cv2.LINE_AA)
            cv2.line(img, (int(x_right), 280), (int(x_right), 480), (0,0,255), 3, cv2.LINE_AA)
            cv2.line(img, (int(img.shape[1]/2 - error * 200), 380), (int(img.shape[1]/2), 480), (0,255,0), 3, cv2.LINE_AA)
            return error, img
        else:
            error = 0
            return error, img
        
    def callback(self, ros_msg):
        img = self.bridge.imgmsg_to_cv2(ros_msg,desired_encoding='bgr8')
        # Normalizing for all computers to have same initial dimensions
        img = cv2.resize(img, (640,480))
        
        edge_img, transform_only_img = self.prepareImage(img)

        left_points_sorted, right_points_sorted = self.findLanes(edge_img)

        error, transform_only_img = self.calcPositionError(left_points_sorted, right_points_sorted, transform_only_img)
        
        cv2.imshow("Lanes Detected Image", transform_only_img)
        cv2.waitKey(1)
        
        velocity = Twist()
        # Use self.linearVel if using scanSubscriber too!
        velocity.linear.x = self.linearVel
        #velocity.linear.x = 0.4
        velocity.angular.z = error
        self.pub.publish(velocity)

            

def main(args=None):
    rclpy.init(args=args)

    follow_track = FollowTrack()

    print("Starting up lane following node")
    rclpy.spin(follow_track)

if __name__ == '__main__':
    main()
