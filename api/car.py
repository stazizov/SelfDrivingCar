import rospy
from nagib_demo_msgs.msg import Control
import time
import rosgraph

TOPIC_NAME = 'control'
NODE_NAME = 'control_publisher'


class CarAPI:
    def __init__(self):
        self.pub = rospy.Publisher(TOPIC_NAME, Control, queue_size=10)
        rospy.init_node(NODE_NAME, anonymous=True)
        self.wait_for_connections(TOPIC_NAME)

    def go(self, speed, angle=0):
        control = Control(speed, angle)
        self.pub.publish(control)

    def wait_for_connections(self, topic):
        ros_master = rosgraph.Master('/rostopic')
        topic = rosgraph.names.script_resolve_name('rostopic', topic)
        num_subs = 0
        for sub in ros_master.getSystemState()[1]:
            if sub[0] == topic:
                num_subs += 1

        for i in range(10):
            if self.pub.get_num_connections() == num_subs:
                return
            time.sleep(0.1)
        raise RuntimeError("failed to get publisher")
