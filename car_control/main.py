import cv2
from line_estimator.get_center import get_center
from simple_pid import PID

error, rR, lR = 0, 9999999, 9999999
pid = PID(80, 0, 3, setpoint=0)
pid.output_limits = (-90, 90)

MAX_SPEED = 20

def image_process(image, sim_api):
    global error, rR, lR
    angle = pid(error)

    speed = MAX_SPEED
    if min([rR, lR]) < 100:
        speed = 1
        
    sim_api.go(speed, angle + 90)

    image = cv2.resize(image, (1280 // 4, 720 // 4))
    error, lR, rR, image = get_center(image)

    image = cv2.resize(image, (1280, 720))
    sim_api.imshow(image)

    # print(min([rR, lR]))
