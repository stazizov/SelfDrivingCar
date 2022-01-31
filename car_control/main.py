#!/usr/bin/env python

import cv2
from line_estimator.get_center import get_center
from simple_pid import PID
from datetime import datetime

IMAGE_PATH = "/catkin_ws/src/nagib_application/car_control/test.png"

error, rR, lR = 0, 9999999, 9999999
pid = PID(80, 0, 3, setpoint=0)
pid.output_limits = (-90, 90)

MAX_SPEED = 20
init_t = datetime.now()

def image_process(image, car):
    global error, rR, lR
    angle = pid(error)

    speed = MAX_SPEED
    if min([rR, lR]) < 100:
        speed = 1
        
    car.go(speed, angle + 90)

    image = cv2.resize(image, (1280 // 4, 720 // 4))
    error, lR, rR, image = get_center(image)
    cv2.imwrite(IMAGE_PATH, image)

    print(min([rR, lR]))
