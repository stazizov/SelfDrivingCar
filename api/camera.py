import rospy
import cv2
import numpy as np
from nagib_demo_msgs.msg import Camera
from api.car import CarAPI


class CameraAPI:
    def __init__(self, image_handler):
        self.image_handler = image_handler
        self.car = CarAPI()

    def startup(self):
        # rospy.init_node('image_listener', anonymous=True)
        rospy.Subscriber('image', Camera, self.image_converter)
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()
    def data_uri_to_cv2_img(self, uri):
        encoded_data = uri
        nparr = np.fromstring(encoded_data.decode('base64'), np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        return img

    def image_converter(self, image):
        img = self.data_uri_to_cv2_img(image.image)
        self.image_handler(img, self.car)
