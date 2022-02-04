#!/usr/bin/python

from api.simulator import SimulatorAPI
import rospy
import numpy as np
from api.camera import CameraAPI
from car_control.main import image_process

print("hello buenos")

if __name__ == '__main__':
    try:
        sim_api = SimulatorAPI()
        camera = CameraAPI(image_process, sim_api)
        camera.startup()
    except rospy.ROSInterruptException:
        pass
