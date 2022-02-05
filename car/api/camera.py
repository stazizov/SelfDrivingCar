import rclpy
import cv2
import numpy as np
from car_msgs.msg import Camera
from ..api.simulator import SimulatorAPI
from rclpy.node import Node
import base64

class CameraAPI(Node):
    def __init__(self, image_handler, sim_api):
        super().__init__('camera_api')
        self.image_handler = image_handler
        self.sim_api = sim_api
        self.subscription = self.create_subscription(
            Camera,
            'image',
            self.image_converter,
            10)
        self.subscription  # prevent unused variable warning

    def data_uri_to_cv2_img(self, uri):
        encoded_data = uri
        nparr = np.fromstring(base64.b64decode(uri), np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        return img

    def image_converter(self, msg):
        img = self.data_uri_to_cv2_img(msg.image)
        self.image_handler(img, self.sim_api)
