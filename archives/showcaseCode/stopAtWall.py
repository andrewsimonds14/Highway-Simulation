#!/usr/bin/env python3
import rclpy , cv2, cv_bridge, numpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose
import time

class stopAtWallNode(Node):
    def __init__(self):
        super().__init__("stopAtWall")
        self.velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.handle_camera_data, 10)
        self.location_subscriber = self.create_subscription(Odometry, '/odom', self.subscribe_pos, 10)


    def handle_camera_data(self,msg):
        global perr, ptime, serr, dt

        image0 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        image1 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='mono8')

        rows,cols, x = image0.shape
        size = rows*cols

        mask = cv2.inRange(image1,0,150)
        output = cv2.bitwise_and(image1,image1,mask = mask)

        numberOfNonFloorPixels = cv2.countNonZero(output)

        #Check if most of the pixels fall within the mask range...
        if((size * 0.9) < numberOfNonFloorPixels):
            self.publish_speed(-0.1, -0.4)
            print('Wall detected')
            time.sleep(5)
        elif(y_robot < -4):
            self.publish_speed(0.0,1.5)
            print('Outside of range')
            time.sleep(2)
            self.publish_speed(0.2, 0.0)
            time.sleep(3)
        else:
            self.publish_speed(0.2, 0.0)


        cv2.imshow("original_image", image1)
        cv2.waitKey(2)
    
    def publish_speed(self, linearVel, angularVel):
        velocity = Twist()
        velocity.linear.x = linearVel
        velocity.angular.z = angularVel
        self.velocity_publisher.publish(velocity)

    def subscribe_pos(self,msg):
        global x_robot, y_robot
        x_robot = msg.pose.pose.position.x
        y_robot = msg.pose.pose.position.y # If this is less than -0.4 act as if its a wall
        #print(x_robot)
        #print(y_robot)

def main(args=None):
    rclpy.init(args=args)
    node1 = stopAtWallNode()
    rclpy.spin(node1)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

    