from car_msgs.msg import Camera, Control, Display, Encoder
import base64
import cv2
from rclpy.node import Node
import numpy as np

CAR_TOPIC_NAME = 'control'
CAR_NODE_NAME = 'control_publisher'

DISPLAY_TOPIC_NAME = 'display'


class SimulatorAPI(Node):
    def __init__(self, image_handler):
        super().__init__('simulator_api')
        self.car_pub = self.create_publisher(Control, CAR_TOPIC_NAME, 10)
        self.display_pub = self.create_publisher(
            Display, DISPLAY_TOPIC_NAME, 10)

        self.subscription = self.create_subscription(
            Encoder,
            'encoder',
            self.update_encoder,
            10)
        self.encoder_handler = None

        self.subscription = self.create_subscription(
            Camera,
            'image',
            self.image_converter,
            10)
        self.image_handler = image_handler

    def go(self, speed, angle=0):
        control = Control()
        control.speed = speed
        control.rotation = int(angle)
        self.car_pub.publish(control)

    def imshow(self, image):
        image = cv2.resize(image, (1280, 720))
        base64_image = base64.b64encode(
            cv2.imencode('.jpg', image)[1]).decode()
        display = Display()
        display.image = str(base64_image)
        self.display_pub.publish(display)

    def update_encoder(self, msg):
        if self.encoder_handler is not None:
            self.encoder_handler(msg.left, msg.right)

    def data_uri_to_cv2_img(self, uri):
        nparr = np.fromstring(base64.b64decode(uri), np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
        return img

    def image_converter(self, msg):
        img = self.data_uri_to_cv2_img(msg.image)
        self.image_handler(img, self)
