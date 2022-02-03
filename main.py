#!/usr/bin/python

import rospy
import numpy as np
from api.camera import CameraAPI
from car_control.main import image_process

print("hello buenos")

if __name__ == '__main__':
    try:
        camera = CameraAPI(image_process)
        camera.startup()
    except rospy.ROSInterruptException:
        pass
