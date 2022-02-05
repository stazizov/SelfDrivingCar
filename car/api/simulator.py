import rclpy
from car_msgs.msg import Control, Display
import time
import base64
import cv2
from rclpy.node import Node

CAR_TOPIC_NAME = 'control'
CAR_NODE_NAME = 'control_publisher'

DISPLAY_TOPIC_NAME = 'display'

class SimulatorAPI(Node):
    def __init__(self):
        super().__init__('simulator_api')
        self.car_pub = self.create_publisher(Control, CAR_TOPIC_NAME, 10)
        self.display_pub = self.create_publisher(Display, DISPLAY_TOPIC_NAME, 10)

    def go(self, speed, angle=0):
        control = Control()
        control.speed = speed
        control.rotation = int(angle)
        self.car_pub.publish(control)

    def imshow(self, image):
        image = cv2.resize(image, (1280, 720))
        base64_image = base64.b64encode(cv2.imencode('.jpg', image)[1]).decode()
        display = Display()
        display.image = str(base64_image)
        self.display_pub.publish(display)
