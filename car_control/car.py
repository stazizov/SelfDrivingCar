from enum import Enum
import cv2
from cv.RoadDetector import RoadDetector
from simple_pid import PID


class TurnDirection(Enum):
    Left = 1
    Right = 2


class Car:
    '''
    Main Car conrol class
    '''

    def __init__(self,
                 road_detector,
                 PID_settings,
                 max_speed,
                 min_speed
                 ):
        self.road_detector = road_detector
        self.sim_api = None

        self.PID = PID(*PID_settings, setpoint=0)
        self.PID.output_limits = (-90, 90)

        self.last_error = 0
        self.curveness = None

        self.MAX_SPEED = max_speed
        self.MIN_SPEED = min_speed

    def follow_road(self, image):
        angle = self.PID(self.last_error)
        speed = self.MAX_SPEED

        if self.curveness is not None and min(self.curveness) < 100:
            speed = self.MIN_SPEED

        print(speed, angle)
        self.sim_api.go(speed, angle)

        image = cv2.resize(image, self.road_detector.img_size)
        image, self.last_error, self.curvenes = self.road_detector.forward(image)
        return image

    def turn(self, direction):
        self.last_error = 0
        self.curveness = None
        # TODO: Implement turn logic
        if direction == TurnDirection.Left:
            pass
        elif direction == TurnDirection.Right:
            pass
