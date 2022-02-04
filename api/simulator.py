import rospy
from nagib_demo_msgs.msg import Control, Display
import time
import rosgraph
import base64
import cv2

CAR_TOPIC_NAME = 'control'
CAR_NODE_NAME = 'control_publisher'

DISPLAY_TOPIC_NAME = 'display'

class SimulatorAPI:
    def __init__(self):
        self.car_pub = rospy.Publisher(CAR_TOPIC_NAME, Control, queue_size=10)
        rospy.init_node(CAR_NODE_NAME, anonymous=True)
        self.display_pub = rospy.Publisher(DISPLAY_TOPIC_NAME, Display, queue_size=10)
        self.wait_for_connections(CAR_TOPIC_NAME, self.car_pub)
        self.wait_for_connections(DISPLAY_TOPIC_NAME, self.display_pub)

    def go(self, speed, angle=0):
        control = Control(speed, angle)
        self.car_pub.publish(control)

    def imshow(self, image):
        image = cv2.resize(image, (1280, 720))
        base64_image = base64.b64encode(cv2.imencode('.jpg', image)[1]).decode()
        self.display_pub.publish(base64_image)

    def wait_for_connections(self, topic, pub):
        ros_master = rosgraph.Master('/rostopic')
        topic = rosgraph.names.script_resolve_name('rostopic', topic)
        num_subs = 0
        for sub in ros_master.getSystemState()[1]:
            if sub[0] == topic:
                num_subs += 1

        for _ in range(10):
            if pub.get_num_connections() == num_subs:
                return
            time.sleep(0.1)
        raise RuntimeError("failed to get publisher")
