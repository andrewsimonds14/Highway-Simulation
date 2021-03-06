import numpy as np
import cv2
import keras
from keras.models import load_model
from keras.preprocessing import image
import cv_bridge
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image

kerasModelPath = input('Please enter the filePath for the h5 file (Example: /home/brock/Documents/TurtleBotClassifer.h5): ')

class getCameraDataNode(Node):
    def __init__(self):
        super().__init__("getCamDataNode")
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = self.create_subscription(Image, "/camera/image_raw", self.handle_camera_data, 10)
        self.model = keras.models.load_model(kerasModelPath)

    def handle_camera_data(self,msg):
        global perr, ptime, serr, dt
        bot_img = self.bridge.imgmsg_to_cv2(msg,desired_encoding='passthrough')
        bot_img = cv2.resize(bot_img,(640,480))
        img = np.expand_dims(bot_img,axis=0)
        prediction = int(self.model.predict(img))
        if prediction == 0:
            print('TurtleBot Detected')
        if prediction != 0:
            print('No TurtleBot Detected')
        



def main(args=None):
    rclpy.init(args=args)
    node = getCameraDataNode()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()

