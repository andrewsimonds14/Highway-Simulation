#!/usr/bin/env python3
import rclpy , cv2, cv_bridge, numpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class getCameraDataNode(Node):
    def __init__(self):
        super().__init__("getCamDataNode")
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.handle_camera_data, 10)

    def handle_camera_data(self,msg):
        global perr, ptime, serr, dt

        lowerWallBoundaries = numpy.array([0,25,100], dtype="uint8")
        upperWallBoundaries = numpy.array([100,125,200], dtype="uint8")

        image0 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        image1 = self.bridge.imgmsg_to_cv2(msg,desired_encoding='mono8')

        rows,cols, x = image0.shape
        size = rows*cols

        mask = cv2.inRange(image0,lowerWallBoundaries,upperWallBoundaries)
        output = cv2.bitwise_and(image0,image0,mask = mask)

        greyOutput = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)

        numberOfBrownPixels = cv2.countNonZero(greyOutput)

        #Check if most of the pixels fall within the mask range...
        if((size * 0.9) < numberOfBrownPixels):
            print('WALL')

        #transformation
        #img = cv2.resize(image0,None,fx=0.6, fy=0.6, interpolation = cv2.INTER_CUBIC)
        #rows, cols, ch = img.shape
        #pts1 = numpy.float32([[90,122],[313,122],[35,242],[385,242]])
        #pts2 = numpy.float32([[0,0],[400,0],[0,400],[400,400]])
        #M = cv2.getPerspectiveTransform(pts1,pts2)
        #img_size = (img.shape[1], img.shape[0])
        #image = cv2.warpPerspective(img,M,(img_size[0]+100,img_size[1]+100))#img_size
        cv2.imshow("original_image", output)
        #cv2.imshow("transformed_image", image)
        cv2.waitKey(2)

def main(args=None):
    rclpy.init(args=args)
    node = getCameraDataNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

    