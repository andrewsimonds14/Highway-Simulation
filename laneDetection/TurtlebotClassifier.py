#!/usr/bin/env python3

import numpy as np
import cv2
from keras.models import load_model
from keras.preprocessing import image
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
        img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
        model = keras.models.load_model('TurtleBotClassifer.h5')
        bot_img = cv2.resize(bot_img,(640,480))
        img = np.expand_dims(bot_img,axis=0)
        prediction = int(model.predict(img))
        if prediction == 0:
            print('TurtleBot Detected')
        



def main(args=None):
    rclpy.init(args=args)
    node = getCameraDataNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

    